"""Real sensor capture and portable, bounded atomic file transport."""
import os
import json
import time
from pathlib import Path
import numpy as np
import torch
from PIL import Image

DATA_DIR = Path(os.environ.get('GO2_DATA_DIR', Path(__file__).resolve().parents[1] / 'outputs/bridge'))
DATA_DIR.mkdir(parents=True, exist_ok=True)
CMD_VEL_FILE = DATA_DIR / 'cmd_vel.json'

class RobotDataManager:
    def __init__(self, env, lidar_annotators, cameras, cfg):
        self.env, self.cfg = env, cfg
        self.lidar_annotators, self.cameras = lidar_annotators, cameras
        self.step_count = 0
        self.last_cmd = 0
        self.command_time = None
        self.latest = {}
        self.errors = {}

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
        if self.step_count % 3:
            return
        robot = self.env.unwrapped.scene['unitree_go2']
        rs = robot.data.root_state_w.detach().cpu().numpy()
        d = {'timestamp':time.time(), 'frame':self.step_count//3, 'num_envs':len(rs),
             'robots':rs.tolist(), 'odom':{'position':rs[0,:3].tolist(),
             'orientation':rs[0,[4,5,6,3]].tolist(), 'frame_id':'odom', 'child_frame_id':'unitree_go2/base_link'}}
        slot = (self.step_count//3) % 4
        if self.cameras:
            cam = self.cameras[0]
            try:
                rgb = cam.get_rgb()
                depth = cam.get_depth()
                if rgb is not None and rgb.size:
                    name = f'color_{slot}.png'
                    Image.fromarray(rgb.astype(np.uint8)).save(DATA_DIR/name)
                    d['color_image'] = name
                    d['intrinsics'] = cam.get_intrinsics_matrix().tolist()
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
        if self.lidar_annotators:
            try:
                pc=self.lidar_annotators[0].get_data()
                pts=pc.get('data') if isinstance(pc,dict) else pc
                if isinstance(pts,np.ndarray) and pts.ndim==2 and pts.shape[1]>=3 and len(pts):
                    pts=pts[:,:3].astype(np.float32)
                    pts=pts[np.isfinite(pts).all(axis=1)]
                    name=f'lidar_{slot}.npy'
                    np.save(DATA_DIR/name,pts)
                    d['point_cloud']=name
                    self.errors.pop('lidar_format',None)
                elif isinstance(pc,dict):
                    self.errors['lidar_format']=str({k:(getattr(v,'shape',type(v).__name__)) for k,v in pc.items()})
            except Exception as e:
                self.errors['lidar']=str(e)
        d['errors']=self.errors
        tmp=DATA_DIR/'data.json.tmp'
        tmp.write_text(json.dumps(d))
        tmp.replace(DATA_DIR/'data.json')
        self.latest=d

    def destroy_node(self):
        pass
