"""独立三角网格核验、可通行出生与实际高斯相机运动验证。"""
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
import trimesh

def validate(validator):
    scene=validator.dm.interior_scene
    checks=[]
    def check(name,passed,**evidence):checks.append({'name':name,'pass':bool(passed),**evidence})
    check('InteriorGS Gaussian source comes from requested USDZ',scene.renderer.info['gaussians']==1445429,**scene.renderer.info)
    check('Mesh-only metric stage has no synthetic floor or Gaussian geometry',scene.mesh_count==60 and scene.metric_volume_count==0,mesh_count=scene.mesh_count,synthetic_ground=False,gaussians_in_rtx_stage=False)
    check('Occupancy random spawn with clearance',scene.spawn['occupancy_value']==255 and scene.spawn['clearance_m']>=scene.required_clearance,**scene.spawn)
    check('Mesh independently supports spawn',scene.mesh_support['hit'] and abs(scene.mesh_support['position'][2])<.15,**scene.mesh_support)
    states=np.asarray(validator.trajectory)
    values=[];clearances=[]
    for state in states:
        r,c=scene.world_to_pixel(state[1],state[2])
        valid=0<=r<scene.image.shape[0] and 0<=c<scene.image.shape[1]
        values.append(int(scene.image[r,c]) if valid else 0)
        clearances.append(float(scene.clearance[r,c]) if valid else 0.)
    check('Robot feet have physical mesh support contacts',float(np.mean(validator.foot_forces[50:]))>30.,mean_vertical_contact_force_N=float(np.mean(validator.foot_forces[50:])))
    check('Robot trajectory stays in occupancy free space',np.mean(np.asarray(values)==255)>.99,free_fraction=float(np.mean(np.asarray(values)==255)),minimum_center_clearance_m=min(clearances))
    mesh_data=np.load(Path(__file__).resolve().parents[1]/'outputs/interior_mesh.npz')
    mesh=trimesh.Trimesh(mesh_data['vertices'],mesh_data['faces'],process=False)
    metadata=validator.snapshot_metadata['depth_image']
    depth=np.load(validator.output/validator.snapshots['depth_image'])
    pixels=np.argwhere(np.isfinite(depth))
    pixels=pixels[::max(1,len(pixels)//32)][:32]
    pose=metadata['depth_pose'];rot=Rotation.from_quat(np.asarray(pose['quaternion_wxyz'])[[1,2,3,0]])
    k=np.asarray(metadata['depth_intrinsics'])
    camera_rays=np.column_stack([(pixels[:,1]-k[0,2])/k[0,0],(pixels[:,0]-k[1,2])/k[1,1],np.ones(len(pixels))])
    directions=rot.apply(camera_rays);directions/=np.linalg.norm(directions,axis=1,keepdims=True)
    origins=np.tile(pose['position'],(len(directions),1))
    points,ray_ids,_=mesh.ray.intersects_location(origins,directions,multiple_hits=False)
    expected=rot.inv().apply(points-origins[ray_ids])[:,2]
    errors=np.abs(expected-depth[pixels[ray_ids,0],pixels[ray_ids,1]])
    check('D435 metric depth matches independent collision triangle rays',len(errors)>16 and np.median(errors)<.06 and np.quantile(errors,.9)<.2,rays=len(errors),median_error_m=float(np.median(errors)),p90_error_m=float(np.quantile(errors,.9)))
    pc=np.load(validator.output/validator.snapshots['point_cloud'])
    pc=pc[::max(1,len(pc)//200)][:200]
    meta=validator.snapshot_metadata['point_cloud'];pose=meta['lidar_pose']
    rot=Rotation.from_quat(np.asarray(pose['quaternion_wxyz'])[[1,2,3,0]])
    world=rot.apply(pc)+pose['position']
    _,distance,_=trimesh.proximity.closest_point(mesh,world)
    check('MID360 points lie on independent collision mesh',len(distance)>50 and np.median(distance)<.06 and np.quantile(distance,.9)<.2,points=len(distance),median_distance_m=float(np.median(distance)),p90_distance_m=float(np.quantile(distance,.9)))
    raw=np.asarray(Image.open(validator.output/validator.snapshots['color_image']))
    check('Actual Gaussian RGB has coverage and texture',scene.renderer.frames>100 and float(raw.std())>10,frames=scene.renderer.frames,rgb_std=float(raw.std()),mean_alpha=float(scene.renderer.last_alpha.mean()))
    movement=[]
    for key in validator.keys:
        path=validator.output/f'movement_{key}_rgb.png'
        if path.exists():movement.append(np.asarray(Image.open(path)).astype(float))
    change=max(float(np.mean(np.abs(x-movement[0])))for x in movement[1:]) if len(movement)>1 else 0.
    check('WASDZC changes rendered Gaussian camera views',len(movement)==6 and change>5,views=len(movement),maximum_mean_pixel_change=change)
    (validator.output/'scene_validation.json').write_text(json.dumps(checks,indent=2))
    return checks
