"""MID360 覆盖近似：对用户碰撞 mesh 作真实 CUDA 射线求交，不生成假点云。"""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import warp as wp
from go2.sensor_profile import mid360_directions,PROFILE

@wp.kernel(enable_backward=False)
def _cast(mesh:wp.uint64,origins:wp.array(dtype=wp.vec3),directions:wp.array(dtype=wp.vec3),distances:wp.array(dtype=float)):
    i=wp.tid()
    t=float(0.);u=float(0.);v=float(0.);sign=float(0.);normal=wp.vec3();face=int(0)
    if wp.mesh_query_ray(mesh,origins[i],directions[i],70.,t,u,v,sign,normal,face):
        distances[i]=t
    else:
        distances[i]=-1.

class CollisionMeshLidar:
    def __init__(self,env,robot_index=0):
        self.env=env
        self.robot_index=robot_index
        mesh=np.load(Path(__file__).resolve().parents[1]/'outputs/interior_mesh.npz')
        self.mesh=wp.Mesh(points=wp.array(mesh['vertices'],dtype=wp.vec3,device='cuda:0'),indices=wp.array(mesh['faces'].ravel(),dtype=wp.int32,device='cuda:0'))
        self.last_tick=-1
        self.cached={}
        self.rays_cast=0
        self.scan_times=[]

    def get_data(self):
        stamp=float(self.env.unwrapped.sim.current_time)
        tick=int(round(stamp*1e6)*10//1e6)
        if tick==self.last_tick:return self.cached
        self.last_tick=tick
        az,el=mid360_directions(tick)
        a=np.radians(az);e=np.radians(el)
        local=np.column_stack([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)]).astype(np.float32)
        state=self.env.unwrapped.scene['unitree_go2'].data.root_state_w[self.robot_index].detach().cpu().numpy()
        rotation=Rotation.from_quat(state[[4,5,6,3]])
        origin=state[:3]+rotation.apply(PROFILE['lidar']['position'])
        directions=rotation.apply(local).astype(np.float32)
        origins=np.tile(origin,(len(local),1)).astype(np.float32)
        distance=wp.empty(len(local),dtype=float,device='cuda:0')
        wp.launch(_cast,dim=len(local),inputs=[self.mesh.id,wp.array(origins,dtype=wp.vec3,device='cuda:0'),wp.array(directions,dtype=wp.vec3,device='cuda:0'),distance],device='cuda:0')
        r=distance.numpy()
        valid=(r>=.1)&(r<=70.)&np.isfinite(r)
        self.cached={'data':local[valid]*r[valid,None],'distance':r[valid],'azimuth':az[valid],'elevation':el[valid],
                     'info':{'backend':'Warp CUDA exact collision triangle raycast','scan_time':stamp,'rays':len(local),'instantaneous_scan':True}}
        self.rays_cast+=len(local)
        self.scan_times.append(stamp)
        return self.cached

class CollisionMeshDepth:
    """以 D435 深度内参追踪碰撞 mesh，输出轴向米制深度而非灰度图。"""
    def __init__(self,env,rig,mesh):
        self.env=env;self.rig=rig;self.mesh=mesh
        self.last_time=-1.;self.cached=None
        k=rig.get_depth_intrinsics_matrix()
        width,height=PROFILE['camera']['depth_resolution']
        v,u=np.indices((height,width))
        rays=np.column_stack([(u.ravel()-k[0,2])/k[0,0],(v.ravel()-k[1,2])/k[1,1],np.ones(width*height)])
        self.norm=np.linalg.norm(rays,axis=1).astype(np.float32)
        self.local=(rays/self.norm[:,None]).astype(np.float32)
        self.shape=(height,width)

    def get_depth(self):
        stamp=float(self.env.unwrapped.sim.current_time)
        if stamp==self.last_time:return self.cached.copy()
        self.last_time=stamp
        pos,quat=self.rig.depth_camera.get_world_pose(camera_axes='ros')
        pos=pos.detach().cpu().numpy() if hasattr(pos,'detach') else np.asarray(pos)
        quat=quat.detach().cpu().numpy() if hasattr(quat,'detach') else np.asarray(quat)
        rotation=Rotation.from_quat(quat[[1,2,3,0]])
        directions=rotation.apply(self.local).astype(np.float32)
        origins=np.tile(pos,(len(directions),1)).astype(np.float32)
        distance=wp.empty(len(directions),dtype=float,device='cuda:0')
        wp.launch(_cast,dim=len(directions),inputs=[self.mesh.id,wp.array(origins,dtype=wp.vec3,device='cuda:0'),wp.array(directions,dtype=wp.vec3,device='cuda:0'),distance],device='cuda:0')
        z=distance.numpy()/self.norm
        z[(z<.2)|(z>10.)|~np.isfinite(z)]=np.nan
        self.cached=z.reshape(self.shape)
        return self.cached.copy()
