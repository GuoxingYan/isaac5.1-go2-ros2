"""直接读取用户 USDZ 内的 NuRec 高斯，用 CUDA 3DGS 渲染彩色相机。"""
import gzip
import hashlib
import json
import time
import zipfile
from pathlib import Path
import msgpack
import numpy as np
import torch
from scipy.spatial.transform import Rotation

class GaussianRGBRenderer:
    def __init__(self,path):
        from gsplat import rasterization
        self.rasterization=rasterization
        self.path=Path(path).resolve()
        with zipfile.ZipFile(self.path) as z:
            name=next(n for n in z.namelist() if n.endswith('.nurec'))
            packed=gzip.decompress(z.read(name))
        payload=msgpack.unpackb(packed,raw=False,strict_map_key=False)['nre_data']
        state=payload['state_dict']
        def tensor(name):
            key='.gaussians_nodes.gaussians.'+name
            arr=np.frombuffer(state[key],np.float16).reshape(state[key+'.shape']).astype(np.float32)
            return torch.from_numpy(arr).cuda()
        self.means=tensor('positions')
        self.quats=torch.nn.functional.normalize(tensor('rotations'),dim=-1)
        self.scales=torch.exp(tensor('scales'))
        self.opacities=torch.sigmoid(tensor('densities')).flatten()
        dc=tensor('features_albedo')
        rest=tensor('features_specular').reshape(len(dc),15,3)
        self.colors=torch.cat([dc[:,None,:],rest],dim=1).contiguous()
        self.info={'source':str(self.path),'member':name,'gaussians':len(self.means),'sh_degree':3,
            'renderer':'gsplat CUDA anisotropic 3D Gaussian rasterization',
            'coordinate_frame':'NuRec raw tensor XYZ, same unrotated metre/Z-up frame as collision mesh; USD display-axis conversion intentionally omitted',
            'sha256':hashlib.sha256(self.path.read_bytes()).hexdigest(),
            'limits':'Same NuRec Gaussians and SH3, rendered with gsplat rather than the original 3DGRUT unscented/ray renderer. Robot mesh is not composited into Gaussian RGB.'}
        self.frames=0
        self.timings=[]
        self.last_alpha=None
        self.last_pose=None

    @torch.inference_mode()
    def render(self,position,quaternion_wxyz,intrinsics,resolution):
        start=time.perf_counter()
        pos=np.asarray(position,dtype=np.float64)
        q=np.asarray(quaternion_wxyz,dtype=np.float64)
        rotation=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        view=np.eye(4,dtype=np.float32)
        view[:3,:3]=rotation.T
        view[:3,3]=-rotation.T@pos
        k=torch.as_tensor(np.asarray(intrinsics),dtype=torch.float32,device='cuda')[None]
        image,alpha,_=self.rasterization(self.means,self.quats,self.scales,self.opacities,self.colors,
            torch.as_tensor(view,device='cuda')[None],k,int(resolution[0]),int(resolution[1]),
            sh_degree=3,near_plane=.05,far_plane=100.,packed=True)
        rgb=(image[0].clamp(0,1)*255).to(torch.uint8).cpu().numpy()
        self.last_image=rgb.copy()
        self.last_alpha=alpha[0,:,:,0].cpu().numpy()
        self.last_pose={'position':pos.tolist(),'quaternion_wxyz':q.tolist(),'intrinsics':np.asarray(intrinsics).tolist()}
        self.frames+=1
        self.timings.append(time.perf_counter()-start)
        return rgb
