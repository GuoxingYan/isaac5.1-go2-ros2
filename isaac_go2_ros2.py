import os
import sys
import types
import hydra
import time
import math
import argparse
import json
from pathlib import Path
import numpy as np

# Note: 'import torch' is intentionally deferred to after AppLauncher init
# to avoid DLL conflicts with Isaac Sim 5.1's bundled CUDA runtime.

# Use packages installed in the active conda environment; no Windows DLL shims.
os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("ROS_DISTRO", "humble")

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Ubuntu Isaac Sim 5.1 Go2 simulation")
parser.add_argument("--self-test", action="store_true")
parser.add_argument("--os-keyboard", action="store_true")
parser.add_argument("--cmd-test", action="store_true")
parser.add_argument("--max-steps", type=int, default=0)
parser.add_argument("--report-dir", default="reports/our-go2")
parser.add_argument("--policy", choices=["rough", "flat"], default="rough")
parser.add_argument("--spawn-seed", type=int, default=None)
parser.add_argument("--spawn-clearance", type=float, default=0.2)
parser.add_argument("--data-dir", default="data")
parser.add_argument("--path-file", default=None)
parser.add_argument("--no-cruise", action="store_true")
parser.add_argument("--cruise-start-file", default=None, help="Wait until this file exists before starting cruise")
parser.add_argument("--cruise-trace", default=None, help="Write robot truth and waypoint progress as JSONL")
parser.add_argument("--cruise-finished-file", default=None, help="Create this file when a non-loop cruise finishes")
parser.add_argument("--exit-on-cruise-finished", action="store_true", help="Exit after a non-loop cruise finishes")
parser.add_argument("--exit-on-cruise-aborted", action="store_true", help="Exit after cruise safety/stall abort")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments (consume known args, leave remaining for Hydra)
args_cli, _remaining = parser.parse_known_args()

# Restore sys.argv with only remaining args (excluding consumed AppLauncher args)
sys.argv = [sys.argv[0]] + _remaining

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

from go2.go2_env import Go2RSLEnvCfg, camera_follow
import env.sim_env as sim_env
import go2.go2_sensors as go2_sensors
import omni
import carb
import go2.go2_ctrl as go2_ctrl

import ros2.go2_ros2_bridge as go2_ros2_bridge
ROS2_AVAILABLE = True

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):

    # Go2 Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    go2_env_cfg.scene.num_envs = cfg.num_envs
    go2_env_cfg.decimation = math.ceil(1./go2_env_cfg.sim.dt/cfg.freq)
    go2_env_cfg.sim.render_interval = go2_env_cfg.decimation
    # Set simulation device based on --device arg
    go2_env_cfg.sim.device = getattr(args_cli, "device", "cuda:0")
    if not cfg.sensor.enable_imu:
        go2_env_cfg.scene.mid360_imu=None
    interior_scene = None
    scene_path = None
    if cfg.env_name == "interiorgs":
        if args_cli.policy != "flat" or cfg.num_envs != 1:
            raise ValueError("InteriorGS uses flat policy and one robot per scene")
        path_file = args_cli.path_file or getattr(cfg, "path_file", None)
        if path_file and Path(path_file).exists():
            scene_path = json.loads(Path(path_file).read_text())
            if scene_path.get("frame_id", "scene") != "scene":
                raise ValueError("InteriorGS path_file must use frame_id=scene")
            if not scene_path.get("poses"):
                raise ValueError("InteriorGS path_file contains no poses")
            first = scene_path["poses"][0]
            spawn_xy = (float(first["x"]), float(first["y"]))
            spawn_yaw = float(first.get("yaw", 0.0))
        else:
            spawn_xy = spawn_yaw = None
        from env.interiorgs_scene import InteriorGSScene
        interior_scene = InteriorGSScene(
            args_cli.data_dir, args_cli.spawn_seed, args_cli.spawn_clearance,
            spawn_xy=spawn_xy, spawn_yaw=spawn_yaw,
        )
        if scene_path is None:
            scene_path = InteriorGSScene.default_path(args_cli.data_dir, args_cli.spawn_clearance)
            first = scene_path["poses"][0]
            interior_scene = InteriorGSScene(
                args_cli.data_dir, args_cli.spawn_seed, args_cli.spawn_clearance,
                spawn_xy=(float(first["x"]), float(first["y"])),
                spawn_yaw=float(first.get("yaw", 0.0)),
            )
        if scene_path is not None:
            interior_scene.validate_path(scene_path)
        interior_scene.configure(go2_env_cfg)
    go2_ctrl.init_base_vel_cmd(cfg.num_envs)
    env, policy = (go2_ctrl.get_rsl_flat_policy if args_cli.policy == "flat" else go2_ctrl.get_rsl_rough_policy)(go2_env_cfg)

    # Simulation environment
    if (cfg.env_name == "obstacle-dense"):
        sim_env.create_obstacle_dense_env() # obstacles dense
    elif (cfg.env_name == "obstacle-medium"):
        sim_env.create_obstacle_medium_env() # obstacles medium
    elif (cfg.env_name == "obstacle-sparse"):
        sim_env.create_obstacle_sparse_env() # obstacles sparse
    elif (cfg.env_name == "warehouse"):
        sim_env.create_warehouse_env() # warehouse
    elif (cfg.env_name == "warehouse-forklifts"):
        sim_env.create_warehouse_forklifts_env() # warehouse forklifts
    elif (cfg.env_name == "warehouse-shelves"):
        sim_env.create_warehouse_shelves_env() # warehouse shelves
    elif (cfg.env_name == "full-warehouse"):
        sim_env.create_full_warehouse_env() # full warehouse

    # Sensor setup
    sensor_rendering = cfg.sensor.enable_camera or (cfg.sensor.enable_lidar and cfg.env_name != "interiorgs")
    sm = go2_sensors.SensorManager(cfg.num_envs, sensor_rendering) if (cfg.sensor.enable_lidar or cfg.sensor.enable_camera or cfg.sensor.enable_imu) else None
    lidar_annotators = []
    if cfg.sensor.enable_lidar and not interior_scene:
        lidar_annotators = sm.add_rtx_lidar()
    cameras = (sm.add_camera_for_scene(native_annotators=False) if interior_scene else sm.add_camera(cfg.freq)) if cfg.sensor.enable_camera else []
    if cfg.sensor.enable_imu:
        sm.add_imu(env)
    if interior_scene and cfg.sensor.enable_lidar:
        from env.mesh_lidar import CollisionMeshLidar
        lidar_annotators = [CollisionMeshLidar(env)]
    if interior_scene and sm and (cameras or sm.lidar_prims):
        sm.bind_pose_updates(env)

    # Keyboard control
    keyboard_subscription = None
    if not args_cli.headless:
        system_input = carb.input.acquire_input_interface()
        keyboard_subscription = system_input.subscribe_to_keyboard_events(
            omni.appwindow.get_default_app_window().get_keyboard(), go2_ctrl.sub_keyboard_event)

    # ROS2 Bridge (optional)
    if ROS2_AVAILABLE:
        try:
            dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg, sm)
        except Exception as e:
            print(f"[WARN] ROS2 bridge init failed: {e}")
            dm = None
    else:
        dm = None

    # Print controls
    print(f"[INFO] {cfg.num_envs} Go2 robots spawned")
    print(f"[INFO] Press 1/2/3 or F1/F2/F3 to select robot, WASDZC to control")
    print(f"[INFO]       1-9 or F1-F9 select robots 0-8")
    print(f"[INFO]       W:Fwd A:Left S:Back D:Right Z:LeftTurn C:RightTurn")

    # Run simulation
    sim_step_dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    obs, _ = env.reset()
    if interior_scene:
        interior_scene.initialize(env, cameras, lidar_annotators, show_rgb=not args_cli.headless)
        if dm is not None:
            dm.interior_scene = interior_scene
    cruise = None
    if interior_scene and scene_path is not None and not args_cli.no_cruise:
        from go2.path_cruise import ScenePathCruise
        cruise = ScenePathCruise(scene_path, loop=bool(scene_path.get("loop", getattr(cfg, "path_loop", False))),
                                 clearance_check=interior_scene.center_clearance,
                                 required_clearance=args_cli.spawn_clearance)
        print(f"[INFO] Scene path cruise: {cruise.path_file} ({len(cruise.waypoints)} waypoints)", flush=True)
    validator = None
    if args_cli.self_test or args_cli.cmd_test:
        from scripts.validate_runtime import RuntimeValidator
        validator = RuntimeValidator(env, dm, args_cli.report_dir, args_cli.os_keyboard, not args_cli.cmd_test)
    step = 0
    while simulation_app.is_running():
        if validator:
            validator.before_step(step + 1000 if args_cli.cmd_test else step)

        start_time = time.time()
        with torch.inference_mode():
            if sm:
                sm.update_scan(step)
            # Update simulated velocity before the frozen policy consumes the
            # command observation. ROS commands retain priority in the cruise.
            if ROS2_AVAILABLE and dm is not None:
                dm.check_cmd_vel()
            if cruise is not None:
                if args_cli.cruise_start_file and not Path(args_cli.cruise_start_file).is_file():
                    if not go2_ctrl.ros_commands:
                        go2_ctrl.base_vel_cmd_input[0].zero_()
                else:
                    cruise.update(env, go2_ctrl)
            # env.step() returns a cached next-step observation. Recompute
            # without advancing observation history so this cycle's newly
            # selected command reaches the policy immediately.
            obs = env.unwrapped.observation_manager.compute(update_history=False)
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _, _ = env.step(actions)

            # ROS2 data (optional)
            if ROS2_AVAILABLE and dm is not None:
                dm.pub_ros2_data()

            if validator:
                validator.after_step(step + 1000 if args_cli.cmd_test else step)
            step += 1
            if args_cli.cruise_trace and cruise is not None and (step % 5 == 0 or cruise.finished or cruise.aborted):
                state = env.unwrapped.scene["unitree_go2"].data.root_state_w[0].detach().cpu().numpy()
                trace_path = Path(args_cli.cruise_trace)
                trace_path.parent.mkdir(parents=True, exist_ok=True)
                with trace_path.open("a", encoding="utf-8") as trace:
                    trace.write(json.dumps({"wall_time": time.time(), "simulation_time": float(env.unwrapped.sim.current_time),
                                            "position": state[:3].tolist(), "quaternion_wxyz": state[3:7].tolist(),
                                            "velocity_world": state[7:10].tolist(), "waypoint_index": cruise.index,
                                            "waypoint_count": len(cruise.waypoints), "complete": cruise.finished,
                                            "command": cruise.command.tolist(), "aborted": cruise.aborted,
                                            "abort_reason": cruise.abort_reason, "diagnostics": cruise.diagnostics}) + "\n")
            if cruise is not None and cruise.finished and args_cli.cruise_finished_file:
                finished_path = Path(args_cli.cruise_finished_file)
                finished_path.parent.mkdir(parents=True, exist_ok=True)
                finished_path.touch(exist_ok=True)
            if cruise is not None and cruise.finished and args_cli.exit_on_cruise_finished:
                break
            if cruise is not None and cruise.aborted and args_cli.exit_on_cruise_aborted:
                break
            if (validator and validator.finished) or (args_cli.max_steps and step >= args_cli.max_steps):
                break

            if args_cli.max_steps and step == 60 and not args_cli.headless:
                from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                Path(args_cli.report_dir).mkdir(parents=True, exist_ok=True)
                viewport_capture = capture_viewport_to_file(get_active_viewport(), str(Path(args_cli.report_dir).resolve()/'simulation_view.png'))
            # Camera follow
            if (cfg.camera_follow):
                camera_follow(env)

            # limit loop time
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, sim_step_dt/elapsed_time)
        if step % 50 == 0:
            print(f"[STEP {step}] {actual_loop_time*1000:.2f}ms RTF={rtf:.2f}", flush=True)

    if args_cli.max_steps:
        from pxr import Usd, UsdGeom
        root = env.unwrapped.scene["unitree_go2"].data.root_state_w.detach().cpu().numpy()
        checks = []
        def key(name, press=True):
            go2_ctrl.sub_keyboard_event(types.SimpleNamespace(input=name, type=carb.input.KeyboardEventType.KEY_PRESS if press else carb.input.KeyboardEventType.KEY_RELEASE))
        key("W"); key("A"); key("A", False)
        checks.append({"name":"combo release preserves held W", "pass":bool(torch.allclose(go2_ctrl.base_vel_cmd_input[0], torch.tensor([1.,0.,0.])))})
        key("SPACE")
        checks.append({"name":"SPACE stops all robots", "pass":bool((go2_ctrl.base_vel_cmd_input==0).all())})
        if cfg.num_envs > 1:
            key("2"); key("W")
            checks.append({"name":"robot 2 selected exclusively", "pass":bool(go2_ctrl.selected_robot==1 and go2_ctrl.base_vel_cmd_input[1,0]==1 and (go2_ctrl.base_vel_cmd_input[0]==0).all())})
            key("F1")
            checks.append({"name":"F1 selects robot 1 and stops previous", "pass":bool(go2_ctrl.selected_robot==0 and (go2_ctrl.base_vel_cmd_input==0).all())})
        key("SPACE")
        sensor_checks = []
        if cfg.sensor.enable_camera:
            for i, camera in enumerate(cameras):
                rgb, depth = camera.get_rgb(), camera.get_depth()
                sensor_checks.append({"name":f"robot {i} RGBD", "pass":bool(rgb is not None and rgb.size and depth is not None and np.isfinite(depth).any())})
            sensor_checks.append({"name":"all cameras created", "pass":len(cameras)==cfg.num_envs})
        if cfg.sensor.enable_lidar:
            for i, annotator in enumerate(lidar_annotators):
                pc = annotator.get_data()
                pts = pc.get("data") if isinstance(pc,dict) else pc
                sensor_checks.append({"name":f"robot {i} RTX LiDAR", "pass":bool(pts is not None and pts.size)})
            sensor_checks.append({"name":"all LiDARs created", "pass":len(lidar_annotators)==cfg.num_envs})
        if cfg.sensor.enable_imu:
            times=np.array([s['time'] for s in dm.imu_history if s['robot']==0])
            rate=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
            sensor_checks.append({"name":"MID360 IMU 200Hz samples", "pass":bool(len(times)>100 and abs(rate-200)<1), "samples":len(times), "simulation_hz":float(rate)})
        checks.extend(sensor_checks)
        mapping_checks = []
        if interior_scene:
            from go2.sensor_profile import PROFILE

            final_xy = root[0, :2]
            start_xy = np.asarray(interior_scene.spawn["position"][:2], dtype=np.float64)
            displacement = float(np.linalg.norm(final_xy - start_xy))
            cruise_command = cruise.command.astype(float).tolist() if cruise is not None else None
            mapping_checks.append({
                "name": "occupancy waypoint cruise moves the robot",
                "pass": bool(cruise is not None and displacement > 0.05),
                "start_xy": start_xy.tolist(),
                "final_xy": final_xy.astype(float).tolist(),
                "displacement_m": displacement,
                "waypoints": np.asarray(cruise.waypoints, dtype=np.float64)[:, :2].tolist() if cruise is not None else [],
                "final_body_command_vx_vy_omega": cruise_command,
            })

            scan = lidar_annotators[0].get_data() if lidar_annotators else {}
            scan_points = np.asarray(scan.get("data", [])) if isinstance(scan, dict) else np.asarray([])
            scan_timestamps = np.asarray(scan.get("timestamp", []), dtype=np.float64) if isinstance(scan, dict) else np.asarray([])
            scan_span_s = float((scan_timestamps.max() - scan_timestamps.min()) / 1e9) if scan_timestamps.size else 0.0
            mapping_checks.append({
                "name": "collision-mesh LiDAR density and per-point time",
                "pass": bool(len(scan_points) > 10000 and 0.09 <= scan_span_s <= 0.11),
                "returns": int(len(scan_points)),
                "planned_rays": int(scan.get("info", {}).get("rays", 0)) if isinstance(scan, dict) else 0,
                "timestamp_unit": scan.get("info", {}).get("timestamp_unit") if isinstance(scan, dict) else None,
                "timestamp_min_ns": float(scan_timestamps.min()) if scan_timestamps.size else None,
                "timestamp_max_ns": float(scan_timestamps.max()) if scan_timestamps.size else None,
                "timestamp_span_s": scan_span_s,
            })

            imu_times = np.asarray([s["time"] for s in dm.imu_history if s["robot"] == 0], dtype=np.float64) if dm is not None else np.asarray([])
            imu_hz = float((len(imu_times) - 1) / (imu_times[-1] - imu_times[0])) if len(imu_times) > 1 else 0.0
            mapping_checks.append({
                "name": "mapping ROS profile matches expected PointCloud2/QoS contract",
                "pass": bool(PROFILE["ros"]["pointcloud_point_step_bytes"] == 26
                             and PROFILE["ros"]["qos_depth"] == 256
                             and [f["name"] for f in PROFILE["ros"]["pointcloud_fields"]]
                             == ["x", "y", "z", "intensity", "tag", "line", "timestamp"]),
                "frame_id": PROFILE["ros"]["lidar_frame_id"],
                "fields": PROFILE["ros"]["pointcloud_fields"],
                "point_step_bytes": PROFILE["ros"]["pointcloud_point_step_bytes"],
                "qos": {"reliability": "reliable", "depth": PROFILE["ros"]["qos_depth"], "durability": "volatile"},
                "imu_frame_id": PROFILE["ros"]["imu_frame_id"],
                "imu_hz": imu_hz,
            })
        meshes = 0
        warehouse = omni.usd.get_context().get_stage().GetPrimAtPath("/World/Warehouse")
        if warehouse and warehouse.IsValid():
            meshes = sum(prim.IsA(UsdGeom.Mesh) for prim in Usd.PrimRange(warehouse))
        result = {"scene": cfg.env_name, "num_envs": cfg.num_envs, "policy": args_cli.policy,
                  "steps": step, "control_checks": checks, "root_states": root.tolist(), "warehouse_meshes": meshes,
                  "mapping_checks": mapping_checks,
                  "pass": bool(np.isfinite(root).all() and (root[:,2] > .2).all()
                               and ("warehouse" not in cfg.env_name or meshes > 0)
                               and all(c["pass"] for c in checks + mapping_checks))}
        directory = Path(args_cli.report_dir)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "smoke.json").write_text(json.dumps(result, indent=2))
    if ROS2_AVAILABLE and dm is not None:
        dm.destroy_node()
    if interior_scene:
        interior_scene.save(args_cli.report_dir)
    if sm:
        sm.close()
    if validator:
        validator.finish()
    env.close()
    simulation_app.close()
    if validator and not all(r["pass"] for r in validator.results):
        raise SystemExit(1)
    if args_cli.max_steps and not result["pass"]:
        raise SystemExit(1)

if __name__ == "__main__":
    run_simulator()
