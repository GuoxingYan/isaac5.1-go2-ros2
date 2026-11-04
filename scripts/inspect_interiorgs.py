"""独立 USD 工具进程：导出 mesh 三角形并核对 occupancy 坐标映射。"""
import json
import hashlib
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from pxr import Usd,UsdGeom

root=Path(__file__).resolve().parents[1]
(root/'outputs').mkdir(exist_ok=True)
(root/'reports/interiorgs').mkdir(parents=True,exist_ok=True)
stage=Usd.Stage.Open(str(root/'data/839873_collision.usd'))
vertices=[];faces=[];count=0
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Mesh):continue
    mesh=UsdGeom.Mesh(prim)
    points=np.array(mesh.GetPointsAttr().Get(),dtype=np.float64)
    matrix=np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    points=(np.column_stack([points,np.ones(len(points))])@matrix)[:,:3]
    indices=np.array(mesh.GetFaceVertexIndicesAttr().Get());counts=mesh.GetFaceVertexCountsAttr().Get()
    local=[];start=0
    for n in counts:
        poly=indices[start:start+n]
        for j in range(1,n-1):local.append([poly[0],poly[j],poly[j+1]])
        start+=n
    faces.append(np.array(local,dtype=np.int32)+count);vertices.append(points.astype(np.float32));count+=len(points)
v=np.concatenate(vertices);f=np.concatenate(faces)
np.savez_compressed(root/'outputs/interior_mesh.npz',vertices=v,faces=f)
meta=json.loads((root/'data/occupancy.json').read_text());image=np.asarray(Image.open(root/'data/occupancy.png'))
tri=v[f];relevant=tri[(tri[:,:,2].max(1)>.1)&(tri[:,:,2].min(1)<1.)]
checks=[]
for flipx in [False,True]:
 for flipy in [False,True]:
    canvas=Image.new('L',(image.shape[1],image.shape[0]));draw=ImageDraw.Draw(canvas)
    for t in relevant:
        xy=(t[:,:2]-np.array(meta['lower'][:2]))/meta['scale']
        if flipx:xy[:,0]=image.shape[1]-1-xy[:,0]
        if flipy:xy[:,1]=image.shape[0]-1-xy[:,1]
        draw.polygon([tuple(p)for p in xy],fill=255)
    occupied=np.asarray(canvas)>0
    precision=float(np.mean(image[occupied]==0))
    free_conflict=float(np.mean(occupied[image==255]))
    checks.append({'flip_x':flipx,'flip_y':flipy,'occupied_precision':precision,'free_conflict':free_conflict})
    canvas.save(root/f'reports/interiorgs/mesh_occupancy_{int(flipx)}{int(flipy)}.png')
print('mesh',v.shape,f.shape,'bounds',v.min(0),v.max(0));print(checks)
(root/'reports/interiorgs/mesh_inspection.json').write_text(json.dumps({'mesh_count':len(vertices),'vertices':len(v),'triangles':len(f),'bounds':[v.min(0).tolist(),v.max(0).tolist()],'occupancy_mapping_checks':checks,'source_sha256':{name:hashlib.sha256((root/'data'/name).read_bytes()).hexdigest() for name in ['839873_collision.usd','occupancy.png','occupancy.json']}},indent=2))
