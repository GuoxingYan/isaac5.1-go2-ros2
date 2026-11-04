"""Offline occupancy sampling cross-checked against independent triangle floor rays."""
import json
from pathlib import Path
import numpy as np
import trimesh
from env.interiorgs_scene import InteriorGSScene


def validate(data_dir='data', count=32, output='reports/interiorgs/spawn_sweep.json'):
    cached=np.load('outputs/interior_mesh.npz')
    mesh=trimesh.Trimesh(cached['vertices'],cached['faces'],process=False)
    samples=[]
    for seed in range(count):
        scene=InteriorGSScene(data_dir,seed,.8)
        spawn=dict(scene.spawn)
        x,y,_=spawn['position']
        points,_,_=mesh.ray.intersects_location([[x,y,1.]],[[0.,0.,-1.]],multiple_hits=False)
        spawn['mesh_floor_z']=float(points[0,2]) if len(points) else None
        spawn['pass']=bool(len(points) and abs(points[0,2])<.01 and spawn['occupancy_value']==255 and spawn['clearance_m']>=.8)
        samples.append(spawn)
    result={'seeds':count,'unique_spawn_pixels':len({tuple(x['pixel_row_col'])for x in samples}),
            'mesh_floor_hits':sum(x['mesh_floor_z'] is not None for x in samples),
            'maximum_floor_z_error_m':max(abs(x['mesh_floor_z']) for x in samples if x['mesh_floor_z'] is not None),
            'all_occupancy_white':all(x['occupancy_value']==255 for x in samples),
            'minimum_clearance_m':min(x['clearance_m']for x in samples),'pass':all(x['pass']for x in samples),'samples':samples}
    Path(output).write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items()if k!='samples'}))
    return result

if __name__=='__main__':
    validate()
