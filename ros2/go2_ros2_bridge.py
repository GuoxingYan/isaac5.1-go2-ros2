"""Real sensor capture and portable, bounded atomic file transport."""
import os
import json
import time
from pathlib import Path
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from go2.sensor_profile import PROFILE
from PIL import Image

DATA_DIR = Path(os.environ.get('GO2_DATA_DIR', Path(__file__).resolve().parents[1] / 'outputs/our-go2-bridge'))
DATA_DIR.mkdir(parents=True, exist_ok=True)
CMD_VEL_FILE = DATA_DIR / 'cmd_vel.json'

class RobotDataManager:
    def __init__(self, env, lidar_annotators, cameras, cfg, sensor_manager=None):
        self.env, self.cfg = env, cfg
        self.sensor_manager=sensor_manager
        self.imu_history=[]
        self.imu_cursor=0
        self.last_imu_time={}
        self.camera_history=[]
        self.lidar_history=[]
        self.previous_sensor_data={}
        self.last_camera_tick=-1
        self.last_lidar_tick=-1
        self.lidar_annotators, self.cameras = lidar_annotators, cameras
        self.step_count = 0
        self.last_cmd = 0
        self.command_time = None
        self.latest = {}
        self.errors = {}
        self.cruise = None
        self.overlay_last_step = -10

    def check_cmd_vel(self):
        import go2.go2_ctrl as ctrl
        try:
            mtime = CMD_VEL_FILE.stat().st_mtime_ns
            if mtime != self.last_cmd:
                cmd = json.loads(CMD_VEL_FILE.read_text())
                if abs(time.time() - cmd.get('timestamp', 0)) < 2.0:
                    i = int(cmd.get('robot', ctrl.selected_robot))
                    if 0 <= i < ctrl.base_vel_cmd_input.shape[0]:
                        ctrl.set_ros_command(i, [cmd.get(k, 0.) for k in ('linear_x','linear_y','angular_z')])
                        print('[CMD_VEL APPLIED]',cmd,flush=True)
                self.last_cmd = mtime
            ctrl.expire_ros_commands()
        except (OSError, ValueError, KeyError):
            pass

    def pub_ros2_data(self):
        self.step_count += 1
        robot = self.env.unwrapped.scene['unitree_go2']
        rs = robot.data.root_state_w.detach().cpu().numpy()
        root = rs[0]
        orientation_xyzw = root[[4, 5, 6, 3]]
        rotation = Rotation.from_quat(orientation_xyzw)
        velocity_world = root[7:10]
        angular_velocity_world = root[10:13]
        velocity_body = rotation.inv().apply(velocity_world)
        angular_velocity_body = rotation.inv().apply(angular_velocity_world)
        d = {'timestamp':time.time(), 'simulation_time':float(self.env.unwrapped.sim.current_time), 'frame':self.step_count, 'num_envs':len(rs),
             'robots':rs.tolist(), 'odom':{'position':root[:3].tolist(),
             'orientation':orientation_xyzw.tolist(),
             'linear_velocity_world':velocity_world.tolist(),
             'angular_velocity_world':angular_velocity_world.tolist(), 'frame_id':'odom',
             'child_frame_id':PROFILE['ros'].get('body_frame_id', 'body')}}
        euler = rotation.as_euler('xyz')
        d['robot_state'] = {
            'position_m': root[:3].tolist(),
            'orientation_xyzw': orientation_xyzw.tolist(),
            'rpy_rad': euler.tolist(),
            'rpy_deg': np.rad2deg(euler).tolist(),
            'linear_velocity_world_m_s': velocity_world.tolist(),
            'angular_velocity_world_rad_s': angular_velocity_world.tolist(),
            'linear_velocity_body_m_s': velocity_body.tolist(),
            'angular_velocity_body_rad_s': angular_velocity_body.tolist(),
            'body_up_z': float(rotation.as_matrix()[2, 2]),
        }
        if getattr(self, 'cruise_path_data', None):
            d['navigation_path'] = {
                'frame_id': self.cruise_path_data.get('frame_id', 'scene'),
                'loop': bool(self.cruise_path_data.get('loop', False)),
                'poses': self.cruise_path_data.get('poses', []),
            }
        if self.cruise is not None:
            d['cruise'] = {
                'state': self.cruise.diagnostics.get('state', 'waiting'),
                'waypoint_index': int(self.cruise.index),
                'waypoint_count': len(self.cruise.waypoints),
                'completed_loops': int(self.cruise.completed_loops),
                'target_xy_m': self.cruise.waypoints[self.cruise.index][:2].tolist(),
                'command_body_vx_vy_omega': self.cruise.command.astype(float).tolist(),
                'finished': bool(self.cruise.finished),
                'aborted': bool(self.cruise.aborted),
                'abort_reason': self.cruise.abort_reason,
                'diagnostics': self.cruise.diagnostics,
            }
        if (getattr(self, 'interior_scene', None) and getattr(self, 'cruise_path_data', None)
                and self.step_count - self.overlay_last_step >= 10):
            try:
                from go2.occupancy_overlay import render_occupancy_overlay
                render_occupancy_overlay(
                    self.interior_scene.data_dir / 'occupancy.png', self.interior_scene.meta,
                    self.cruise_path_data, DATA_DIR / 'occupancy_overlay.png',
                    robot_pose=[root[0], root[1], float(rotation.as_euler('zyx')[0])],
                    target_index=self.cruise.index if self.cruise is not None else None,
                    current_index=((self.cruise.index - 1) % len(self.cruise.waypoints)
                                   if self.cruise is not None else None),
                )
                self.overlay_last_step = self.step_count
            except Exception as exc:
                self.errors['occupancy_overlay'] = str(exc)
            else:
                self.errors.pop('occupancy_overlay', None)
        d['sensor_profile']=PROFILE
        if hasattr(self, 'interior_scene'):
            d['scene_sources'] = self.interior_scene.evidence
        sim_time=d['simulation_time']
        slot = self.step_count % 8
        d.update(self.previous_sensor_data)
        camera_tick=int(round(sim_time*1000000)*30//1000000)
        if self.cameras and camera_tick!=self.last_camera_tick:
            self.last_camera_tick=camera_tick
            cam = self.cameras[0]
            try:
                rgb = cam.get_rgb()
                depth = cam.get_depth()
                if rgb is not None and rgb.size:
                    name = f'color_{slot}.png'
                    Image.fromarray(rgb.astype(np.uint8)).save(DATA_DIR/name)
                    d['color_image'] = name
                    d['intrinsics'] = cam.get_intrinsics_matrix().tolist()
                    d['depth_intrinsics'] = cam.get_depth_intrinsics_matrix().tolist()
                    d['camera_time']=sim_time
                    self.camera_history.append(sim_time)
                if depth is not None and depth.size:
                    name = f'depth_{slot}.npy'
                    np.save(DATA_DIR/name, np.asarray(depth,dtype=np.float32))
                    d['depth_image'] = name
                    d['depth_encoding'] = '32FC1'
                frame = cam.get_current_frame()
                sem = frame.get('semantic_segmentation')
                if isinstance(sem, dict) and np.asarray(sem.get('data',[])).size:
                    name=f'semantic_{slot}.npy'
                    np.save(DATA_DIR/name, sem['data'])
                    d['semantic']=name
                    d['semantic_labels']=sem.get('info',{})
                # World-coordinate RGB-D point cloud, explicitly distinguished from LiDAR.
                if depth is not None and depth.size:
                    pts = cam.get_pointcloud(world_frame=True)
                    if pts is not None and pts.size:
                        name=f'rgbd_points_{slot}.npy'
                        pts=np.asarray(pts).reshape(-1,3)
                        pts=pts[np.isfinite(pts).all(axis=1)]
                        np.save(DATA_DIR/name,pts)
                        d['rgbd_point_cloud']=name
            except Exception as e:
                self.errors['camera']=str(e)
        lidar_tick=int(round(sim_time*1000000)*10//1000000)
        if self.lidar_annotators and lidar_tick!=self.last_lidar_tick:
            self.last_lidar_tick=lidar_tick
            try:
                pc=self.lidar_annotators[0].get_data()
                pts=pc.get('data') if isinstance(pc,dict) else pc
                if isinstance(pts,np.ndarray) and pts.ndim==2 and pts.shape[1]>=3 and len(pts):
                    raw_pts=pts[:,:3].astype(np.float32)
                    ranges=np.linalg.norm(raw_pts,axis=1)
                    valid=np.isfinite(raw_pts).all(axis=1)&(ranges>=.1)&(ranges<=70.)
                    pts=raw_pts[valid]
                    name=f'lidar_{slot}.npy'
                    np.save(DATA_DIR/name,pts)
                    d['point_cloud']=name
                    d['lidar_time']=sim_time
                    self.lidar_history.append(sim_time)
                    if isinstance(pc,dict):
                        aux={}
                        for key in ['intensity','timestamp','tag','line','distance','azimuth','elevation']:
                            value=pc.get(key)
                            if isinstance(value,np.ndarray) and value.size:
                                aux[key]=value[valid] if len(value) == len(valid) else value
                        if aux:
                            aux_name=f'lidar_aux_{slot}.npz'
                            np.savez(DATA_DIR/aux_name,**aux)
                            d['lidar_aux']=aux_name
                            d['lidar_timestamp_unit'] = 'nanoseconds' if 'timestamp' in aux else None
                    self.errors.pop('lidar_format',None)
                elif isinstance(pc,dict):
                    self.errors['lidar_format']=str({k:(getattr(v,'shape',type(v).__name__)) for k,v in pc.items()})
            except Exception as e:
                self.errors['lidar']=str(e)
        if self.sensor_manager:
            fresh=[]
            for sample in self.sensor_manager.imu_samples:
                index=sample['robot']
                if sample['time']>self.last_imu_time.get(index,0):
                    fresh.append(sample)
                    self.last_imu_time[index]=sample['time']
            d['imu_samples']=[s for s in self.sensor_manager.imu_samples if s['time']>sim_time-1.0]
            self.imu_history.extend(fresh)
        self.previous_sensor_data={k:v for k,v in d.items() if k in ['color_image','intrinsics','depth_intrinsics','camera_time','depth_image','depth_encoding','semantic','semantic_labels','rgbd_point_cloud','point_cloud','lidar_time','lidar_aux','lidar_timestamp_unit']}
        d['errors']=self.errors
        tmp=DATA_DIR/'data.json.tmp'
        tmp.write_text(json.dumps(d))
        tmp.replace(DATA_DIR/'data.json')
        self.latest=d

    def destroy_node(self):
        pass
