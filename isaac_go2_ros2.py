import os
import sys
import types
import hydra
import time
import math
import argparse

# Note: 'import torch' is intentionally deferred to after AppLauncher init
# to avoid DLL conflicts with Isaac Sim 5.1's bundled CUDA runtime.

# Use packages installed in the active conda environment; no Windows DLL shims.
os.environ.setdefault("ROS_DOMAIN_ID", "42")
os.environ.setdefault("ROS_DISTRO", "foxy")

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Ubuntu Isaac Sim 5.1 Go2 simulation")
parser.add_argument("--self-test", action="store_true")
parser.add_argument("--os-keyboard", action="store_true")
parser.add_argument("--cmd-test", action="store_true")
parser.add_argument("--max-steps", type=int, default=0)
parser.add_argument("--report-dir", default="reports/our-go2")
parser.add_argument("--spawn-seed",type=int,default=None)
parser.add_argument("--spawn-clearance",type=float,default=.8)
parser.add_argument("--data-dir",default="data")
parser.add_argument("--policy", choices=["rough", "flat"], default="rough")

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
    interior_scene=None
    if cfg.env_name=='interiorgs':
        if args_cli.policy!='flat' or cfg.num_envs!=1:
            raise ValueError('InteriorGS uses flat policy and one robot per occupancy-safe spawn')
        from env.interiorgs_scene import InteriorGSScene
        interior_scene=InteriorGSScene(args_cli.data_dir,args_cli.spawn_seed,args_cli.spawn_clearance)
        interior_scene.configure(go2_env_cfg)
        go2_ctrl.linear_speed=.4
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
    sm = go2_sensors.SensorManager(cfg.num_envs, cfg.sensor.enable_lidar or cfg.sensor.enable_camera) if (cfg.sensor.enable_lidar or cfg.sensor.enable_camera or cfg.sensor.enable_imu) else None
    lidar_annotators=[]
    if cfg.sensor.enable_lidar:
        if interior_scene:
            from env.mesh_lidar import CollisionMeshLidar
            lidar_annotators=[CollisionMeshLidar(env)]
        else:
            lidar_annotators=sm.add_rtx_lidar()
    cameras = sm.add_camera(cfg.freq) if cfg.sensor.enable_camera else []
    if cfg.sensor.enable_imu:
        sm.add_imu(env)

    if sm and (cameras or lidar_annotators):
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
        interior_scene.initialize(env,cameras,lidar_annotators,show_rgb=not args_cli.headless)
        dm.interior_scene=interior_scene
    validator = None
    if args_cli.self_test or args_cli.cmd_test:
        from scripts.validate_runtime import RuntimeValidator
        validator = RuntimeValidator(env, dm, args_cli.report_dir, args_cli.os_keyboard, not args_cli.cmd_test)
    step = 0
    while simulation_app.is_running():
        if validator:
            validator.before_step(step + validator.ros_first if args_cli.cmd_test else step)

        start_time = time.time()
        with torch.inference_mode():
            if sm:
                sm.update_scan(step)
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _, _ = env.step(actions)

            # ROS2 data (optional)
            if ROS2_AVAILABLE and dm is not None:
                dm.check_cmd_vel()  # Native Ubuntu ROS2 file transport
                dm.pub_ros2_data()

            if validator:
                validator.after_step(step + validator.ros_first if args_cli.cmd_test else step)
            step += 1
            if (validator and validator.finished) or (args_cli.max_steps and step >= args_cli.max_steps):
                break

            if args_cli.max_steps and step == 60 and not args_cli.headless:
                from pathlib import Path
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
        import json
        from pathlib import Path
        import numpy as np
        from pxr import Usd, UsdGeom
        root = env.unwrapped.scene["unitree_go2"].data.root_state_w.detach().cpu().numpy()
        checks = []
        def key(name, press=True):
            go2_ctrl.sub_keyboard_event(types.SimpleNamespace(input=name, type=carb.input.KeyboardEventType.KEY_PRESS if press else carb.input.KeyboardEventType.KEY_RELEASE))
        key("W"); key("A"); key("A", False)
        checks.append({"name":"combo release preserves held W", "pass":bool(torch.allclose(go2_ctrl.base_vel_cmd_input[0], torch.tensor([go2_ctrl.linear_speed,0.,0.])))})
        key("SPACE")
        checks.append({"name":"SPACE stops all robots", "pass":bool((go2_ctrl.base_vel_cmd_input==0).all())})
        if cfg.num_envs > 1:
            key("2"); key("W")
            checks.append({"name":"robot 2 selected exclusively", "pass":bool(go2_ctrl.selected_robot==1 and go2_ctrl.base_vel_cmd_input[1,0]==go2_ctrl.linear_speed and (go2_ctrl.base_vel_cmd_input[0]==0).all())})
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
                sensor_checks.append({"name":(f"robot {i} MID360 mesh rays" if interior_scene else f"robot {i} RTX LiDAR"), "pass":bool(pts is not None and pts.size)})
            sensor_checks.append({"name":"all LiDARs created", "pass":len(lidar_annotators)==cfg.num_envs})
        if cfg.sensor.enable_imu:
            times=np.array([s['time'] for s in dm.imu_history if s['robot']==0])
            rate=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
            sensor_checks.append({"name":"MID360 IMU 200Hz samples", "pass":bool(len(times)>100 and abs(rate-200)<1), "samples":len(times), "simulation_hz":float(rate)})
        checks.extend(sensor_checks)
        meshes = 0
        warehouse = omni.usd.get_context().get_stage().GetPrimAtPath("/World/Warehouse")
        if warehouse and warehouse.IsValid():
            meshes = sum(prim.IsA(UsdGeom.Mesh) for prim in Usd.PrimRange(warehouse))
        result = {"scene": cfg.env_name, "num_envs": cfg.num_envs, "policy": args_cli.policy,
                  "steps": step, "control_checks": checks, "root_states": root.tolist(), "warehouse_meshes": meshes,
                  "pass": bool(np.isfinite(root).all() and (root[:,2] > .2).all() and ("warehouse" not in cfg.env_name or meshes > 0) and all(c["pass"] for c in checks))}
        directory = Path(args_cli.report_dir)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "smoke.json").write_text(json.dumps(result, indent=2))
    if ROS2_AVAILABLE and dm is not None:
        dm.destroy_node()
    if interior_scene:
        interior_scene.save(args_cli.report_dir)
    if validator:
        validator.finish()
    if sm:
        sm.close()
    env.close()
    simulation_app.close()
    if validator and not all(r["pass"] for r in validator.results):
        raise SystemExit(1)
    if args_cli.max_steps and not result["pass"]:
        raise SystemExit(1)

if __name__ == "__main__":
    run_simulator()
