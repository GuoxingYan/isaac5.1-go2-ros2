"""InteriorGS 原坐标：occupancy 安全出生，碰撞 mesh 仅负责几何与物理。"""
import json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt

class InteriorGSScene:
    def __init__(self,data_dir,seed=None,clearance=.8):
        self.data_dir=Path(data_dir).resolve()
        self.meta=json.loads((self.data_dir/'occupancy.json').read_text())
        self.image=np.asarray(Image.open(self.data_dir/'occupancy.png').convert('L'))
        self.flip_x=True
        self.flip_y=False
        self.scale=float(self.meta['scale'])
        # 该样本用 mesh 投影独立核验四种翻转后，X翻转/Y不翻转吻合度最高。
        mask_path=Path(__file__).resolve().parents[1]/'reports/interiorgs/mesh_occupancy_10.png'
        if not mask_path.exists():
            raise RuntimeError('Run scripts/inspect_interiorgs.py with the isolated USD tools first')
        mesh_occupied=np.asarray(Image.open(mask_path))>0
        self.free=(self.image==255)&~mesh_occupied
        self.clearance=distance_transform_edt(self.free)*self.scale
        self.required_clearance=float(clearance)
        self.seed=int(np.random.SeedSequence().entropy) if seed is None else int(seed)
        rng=np.random.default_rng(self.seed)
        candidates=np.argwhere(self.clearance>=clearance)
        if not len(candidates):
            raise RuntimeError(f'No occupancy/mesh-safe spawn with clearance {clearance}; maximum {self.clearance.max()}')
        row,col=candidates[rng.integers(len(candidates))]
        xy=self.pixel_to_world(row,col)
        self.spawn={'seed':self.seed,'pixel_row_col':[int(row),int(col)],'position':[float(xy[0]),float(xy[1]),.42],
                    'yaw_rad':float(rng.uniform(-np.pi,np.pi)),'clearance_m':float(self.clearance[row,col]),
                    'required_clearance_m':clearance,'candidates':len(candidates),'occupancy_value':int(self.image[row,col]),
                    'mapping':{'flip_x':True,'flip_y':False,'scale':self.scale,'origin_xy':self.meta['min'][:2]}}
        self.renderer=None
        self.mesh_support={}
        self.collisions=[]

    def pixel_to_world(self,row,col):
        x=self.meta['min'][0]+(self.image.shape[1]-col-.5)*self.scale
        y=self.meta['min'][1]+(row+.5)*self.scale
        return np.array([x,y])

    def world_to_pixel(self,x,y):
        col=int(np.floor(self.image.shape[1]-(x-self.meta['min'][0])/self.scale))
        row=int(np.floor((y-self.meta['min'][1])/self.scale))
        return row,col

    def configure(self,cfg):
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg
        from scipy.spatial.transform import Rotation
        cfg.sim.enable_scene_query_support=True
        cfg.scene.ground=None
        cfg.scene.height_scanner=None
        cfg.observations.policy.height_scan=None
        cfg.scene.interior_mesh=AssetBaseCfg(prim_path='/World/InteriorMesh',
            spawn=sim_utils.UsdFileCfg(usd_path=str(self.data_dir/'839873_collision.usd')))
        cfg.scene.unitree_go2.init_state.pos=tuple(self.spawn['position'])
        q=Rotation.from_euler('z',self.spawn['yaw_rad']).as_quat()
        cfg.scene.unitree_go2.init_state.rot=tuple(q[[3,0,1,2]].tolist())
        cfg.viewer.eye=np.array(self.spawn['position'])+np.array([-.6,.8,1.8])
        cfg.viewer.lookat=self.spawn['position']

    def initialize(self,env,cameras,lidars,show_rgb=True):
        import omni.physx
        from pxr import Usd,UsdGeom
        import omni.usd
        from env.gaussian_rgb import GaussianRGBRenderer
        stage=omni.usd.get_context().get_stage()
        mesh_prim=stage.GetPrimAtPath('/World/InteriorMesh')
        self.metric_volume_count=sum(p.GetTypeName() in ['Volume','OmniNuRecFieldAsset'] for p in stage.Traverse())
        for prim in Usd.PrimRange(mesh_prim):
            if prim.IsA(UsdGeom.Mesh):
                UsdGeom.Mesh(prim).GetDoubleSidedAttr().Set(True)
        self.mesh_count=sum(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(mesh_prim))
        if stage.GetPrimAtPath('/World/ground').IsValid():
            raise RuntimeError('Synthetic ground must not be present in the InteriorGS scene')
        x,y,_=self.spawn['position']
        hits=[]
        def on_hit(value):
            if '/World/InteriorMesh' in value.collision:
                hits.append({'collision':value.collision,'rigidBody':value.rigid_body,'position':tuple(value.position),'normal':tuple(value.normal),'distance':value.distance})
            return True
        omni.physx.get_physx_scene_query_interface().raycast_all((x,y,1.),(0.,0.,-1.),2.,on_hit)
        hit=min(hits,key=lambda h:h['distance']) if hits else {'hit':False}
        hit['hit']=bool(hits)
        self.mesh_support={'hit':bool(hit['hit']),'position':[float(x)for x in hit.get('position',[])],'normal':[float(x)for x in hit.get('normal',[])],'distance':float(hit.get('distance',0)),'collision':str(hit.get('collision','')),'rigidBody':str(hit.get('rigidBody',''))}
        if not hit['hit'] or '/World/InteriorMesh' not in str(hit.get('collision',hit.get('rigidBody',''))):
            raise RuntimeError(f'Spawn not supported by collision mesh: {self.mesh_support}')
        if abs(hit['position'][2])>.15:
            raise RuntimeError(f'Spawn support height invalid: {self.mesh_support}')
        self.cameras=cameras
        self.lidars=lidars
        self.renderer=GaussianRGBRenderer(self.data_dir/'839873.usdz')
        from env.mesh_lidar import CollisionMeshDepth
        for rig in cameras:
            rig.gaussian_renderer=self.renderer
            rig.depth_provider=CollisionMeshDepth(env,rig,lidars[0].mesh)
        self.evidence={'scene_id':'839873','spawn':self.spawn,'mesh_count':self.mesh_count,'support':self.mesh_support,
            'rgb_source':'USDZ/NuRec Gaussian parameters → gsplat SH3', 'lidar_backend':'Warp CUDA exact collision mesh ray queries, instantaneous 20k-ray scans at 10Hz; no proprietary motion-distortion or reflectance model', 'metric_source':'839873_collision.usd triangles -> Warp CUDA pinhole depth and LiDAR; authored PhysX colliders for contacts; no Gaussian geometry or synthetic ground in metric queries',
            'renderer':self.renderer.info,'synthetic_ground':False,'metric_stage_volume_count':self.metric_volume_count}
        if show_rgb:
            import omni.ui as ui
            self.window=ui.Window('D435 RGB | InteriorGS Gaussian',width=660,height=410)
            self.provider=ui.ByteImageProvider()
            with self.window.frame:
                with ui.VStack():
                    ui.Label('RGB: 839873.usdz Gaussians | Depth/LiDAR: collision mesh',height=24)
                    ui.ImageWithProvider(self.provider,height=360)
            cameras[0].rgb_display=self.provider

    def save(self,directory):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        if hasattr(self.renderer,'last_image'):
            Image.fromarray(self.renderer.last_image).save(directory/'scene_rgb.png')
            np.save(directory/'scene_depth.npy',self.cameras[0].depth_provider.cached)
            np.save(directory/'scene_lidar.npy',self.lidars[0].cached['data'])
        self.evidence['last_camera_pose']=self.renderer.last_pose
        self.evidence['rendered_frames']=self.renderer.frames
        self.evidence['mean_gaussian_render_s']=float(np.mean(self.renderer.timings)) if self.renderer.timings else 0.
        (directory/'scene_evidence.json').write_text(json.dumps(self.evidence,indent=2))
