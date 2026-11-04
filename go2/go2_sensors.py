"""我方 Go2：D435 双视场成像、MID360 RTX 近似扫描及同位置物理 IMU。"""
import math
from pathlib import Path
import omni
import numpy as np
from pxr import Gf, Vt
from go2.sensor_profile import PROFILE, mid360_directions, write_mid360_usda

def _numpy(value):
    return value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)

class D435Rig:
    """分别保留彩色与深度相机，不将不同分辨率/视场伪装为已对齐图像。"""
    def __init__(self, base):
        from isaacsim.sensors.camera import Camera
        import isaacsim.core.utils.numpy.rotations as rotations
        cfg = PROFILE['camera']
        self.rgb_camera = Camera(prim_path=base+'/front_cam', translation=np.array(cfg['position']),
            orientation=rotations.euler_angles_to_quats(np.array(cfg['rpy_deg']),degrees=True), resolution=tuple(cfg['rgb_resolution']))
        self.depth_camera = Camera(prim_path=base+'/d435_depth', translation=np.array(cfg['position']),
            orientation=rotations.euler_angles_to_quats(np.array(cfg['rpy_deg']),degrees=True), resolution=tuple(cfg['depth_resolution']))
        for camera, fov in [(self.rgb_camera,cfg['rgb_hfov_deg']), (self.depth_camera,cfg['depth_hfov_deg'])]:
            camera.initialize()
            camera.set_focal_length(.018)
            camera.set_horizontal_aperture(2*.018*math.tan(math.radians(fov)/2))
            camera.set_clipping_range(.05,100.)
        self.depth_camera.add_distance_to_image_plane_to_frame()
        self.depth_camera.add_semantic_segmentation_to_frame()
        self.depth_camera.add_pointcloud_to_frame(include_unlabelled=True)
        self.last_capture_time = -1.

    def get_rgb(self):
        if hasattr(self,'gaussian_renderer'):
            pos,quat=self.rgb_camera.get_world_pose(camera_axes='ros')
            image=self.gaussian_renderer.render(_numpy(pos),_numpy(quat),self.get_intrinsics_matrix(),PROFILE['camera']['rgb_resolution'])
            if hasattr(self,'rgb_display'):
                rgba=np.dstack([image,np.full(image.shape[:2],255,dtype=np.uint8)])
                self.rgb_display.set_bytes_data(rgba.ravel().tolist(),[image.shape[1],image.shape[0]])
            return image
        return self.rgb_camera.get_rgb()

    def get_depth(self):
        depth = self.depth_provider.get_depth() if hasattr(self,'depth_provider') else self.depth_camera.get_depth()
        if depth is None:
            return None
        depth = np.asarray(depth,dtype=np.float32).copy()
        cfg=PROFILE['camera']
        depth[(depth<cfg['depth_min_m']) | (depth>cfg['depth_max_m']) | ~np.isfinite(depth)]=np.nan
        return depth

    def get_intrinsics_matrix(self):
        return _numpy(self.rgb_camera.get_intrinsics_matrix())

    def get_depth_intrinsics_matrix(self):
        return _numpy(self.depth_camera.get_intrinsics_matrix())

    def get_current_frame(self):
        return {} if hasattr(self,'depth_provider') else self.depth_camera.get_current_frame()

    def get_pointcloud(self, world_frame=True):
        """由已截断的米制深度反投影，避免将 10 米外几何混入 D435 点云。"""
        from scipy.spatial.transform import Rotation
        depth=self.get_depth()
        if depth is None:
            return None
        v,u=np.indices(depth.shape)
        k=self.get_depth_intrinsics_matrix()
        valid=np.isfinite(depth)
        z=depth[valid]
        pts=np.column_stack([(u[valid]-k[0,2])*z/k[0,0], (v[valid]-k[1,2])*z/k[1,1],z])
        if world_frame:
            pos, quat=self.depth_camera.get_world_pose(camera_axes='ros')
            pts=Rotation.from_quat(_numpy(quat)[[1,2,3,0]]).apply(pts)+_numpy(pos)
        return pts.astype(np.float32)

class SensorManager:
    def __init__(self,num_envs,enable_render=True):
        from isaacsim.core.utils.extensions import enable_extension
        import carb
        if enable_render:
            for extension in ['isaacsim.sensors.camera','isaacsim.sensors.rtx']:
                enable_extension(extension)
            carb.settings.get_settings().set_bool('/isaaclab/render/rtx_sensors',True)
        self.num_envs=num_envs
        self.render_products=[]
        self.lidar_prims=[]
        self.imus=[]
        self.imu_samples=[]
        self.imu_last_time={}
        self.scan_index=-1
        self.profile=PROFILE

    def add_rtx_lidar(self):
        import omni.replicator.core as rep
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.xforms import reset_and_set_xform_ops
        path=Path(__file__).resolve().parents[1]/'assets/sensors/mid360.usda'
        write_mid360_usda(path)
        annotators=[]
        for i in range(self.num_envs):
            prim=add_reference_to_stage(str(path),f'/World/OurSensors/env_{i}/lidar',prim_type='OmniLidar')
            reset_and_set_xform_ops(prim.GetPrim(),Gf.Vec3d(*PROFILE['lidar']['position']),Gf.Quatd(1,0,0,0))
            self.lidar_prims.append(prim.GetPrim())
            texture=rep.create.render_product(prim.GetPath(),[32,32],name=f'MID360_{i}',render_vars=['GenericModelOutput','RtxSensorMetadata'])
            self.render_products.append(texture)
            annotator=rep.AnnotatorRegistry.get_annotator('IsaacExtractRTXSensorPointCloudNoAccumulator')
            annotator.initialize(transformPoints=False,outputDistance=True,outputIntensity=True,outputTimestamp=True,outputAzimuth=True,outputElevation=True)
            annotator.attach(texture.path)
            annotators.append(annotator)
        return annotators

    def update_scan(self,step):
        """每个扫描窗口变更射线相位；几何求交仍由 RTX 执行。"""
        index=step//5
        if index==self.scan_index:
            return
        self.scan_index=index
        az,el=mid360_directions(index)
        for prim in self.lidar_prims:
            prim.GetAttribute('omni:sensor:Core:emitterState:s001:azimuthDeg').Set(Vt.FloatArray.FromNumpy(az))
            prim.GetAttribute('omni:sensor:Core:emitterState:s001:elevationDeg').Set(Vt.FloatArray.FromNumpy(el))

    def add_camera(self,freq):
        self.cameras=[D435Rig(f'/World/OurSensors/env_{i}') for i in range(self.num_envs)]
        return self.cameras

    def bind_pose_updates(self,env):
        """每次渲染前从物理根节点读取位姿，避免 USD 初始位姿被外部渲染器误用。"""
        self.env=env
        sim=env.unwrapped.sim
        original=sim.render
        self.original_render=original
        def render_with_sensor_pose(*args,**kwargs):
            self.sync_poses()
            return original(*args,**kwargs)
        sim.render=render_with_sensor_pose
        self.sync_poses()

    def sync_poses(self):
        from scipy.spatial.transform import Rotation
        robot=self.env.unwrapped.scene['unitree_go2']
        transforms=robot.root_physx_view.get_root_transforms().detach().cpu().numpy()
        for i,state in enumerate(transforms):
            rotation=Rotation.from_quat(state[3:7])
            if hasattr(self,'cameras'):
                camera_rotation=rotation*Rotation.from_euler('xyz',PROFILE['camera']['rpy_deg'],degrees=True)
                q=camera_rotation.as_quat()[[3,0,1,2]]
                pos=state[:3]+rotation.apply(PROFILE['camera']['position'])
                for camera in [self.cameras[i].rgb_camera,self.cameras[i].depth_camera]:
                    camera.set_world_pose(position=pos.astype(np.float32),orientation=q.astype(np.float32),camera_axes='world')
            if i<len(self.lidar_prims):
                lidar_rotation=rotation*Rotation.from_euler('xyz',PROFILE['lidar']['rpy_deg'],degrees=True)
                q=lidar_rotation.as_quat()[[3,0,1,2]]
                pos=state[:3]+rotation.apply(PROFILE['lidar']['position'])
                self.lidar_prims[i].GetAttribute('xformOp:translate').Set(Gf.Vec3d(*pos.tolist()))
                self.lidar_prims[i].GetAttribute('xformOp:orient').Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*q[1:].tolist())))

    def add_imu(self,env):
        self.imu=env.unwrapped.scene['mid360_imu']
        self.imus=[self.imu] * self.num_envs
        self.sim=env.unwrapped.sim
        self.sim.add_physics_callback('mid360_imu_200hz',self._sample_imu)

    def _sample_imu(self,dt):
        """以 5ms 物理步读取 Isaac Lab IMU，包含偏置处的角运动加速度。"""
        self.imu.update(dt,force_recompute=True)
        data=self.imu.data
        stamp=float(self.sim.current_time)
        acceleration=data.lin_acc_b.detach().cpu().numpy()
        angular=data.ang_vel_b.detach().cpu().numpy()
        for i in range(self.num_envs):
            if stamp<=self.imu_last_time.get(i,0):
                continue
            self.imu_last_time[i]=stamp
            self.imu_samples.append({'robot':i,'time':stamp,'linear_acceleration':acceleration[i].tolist(),
                'angular_velocity':angular[i].tolist(),'frame_id':'unitree_go2/mid360_imu'})
        self.imu_samples=self.imu_samples[-20000:]

    def close(self):
        if hasattr(self,'original_render'):
            self.env.unwrapped.sim.render=self.original_render
        if hasattr(self,'sim'):
            self.sim.remove_physics_callback('mid360_imu_200hz')
