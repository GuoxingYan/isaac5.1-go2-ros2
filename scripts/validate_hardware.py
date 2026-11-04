"""检查实际采集数据、视场/采样时间和 MID360 IMU 运动响应。"""
import json
from pathlib import Path
import numpy as np
from PIL import Image
from go2.sensor_profile import PROFILE, mid360_directions

def validate(validator):
    checks=[]
    output=validator.output
    dm=validator.dm
    def check(name,passed,**detail):
        checks.append({'name':name,'pass':bool(passed),**detail})
    cam=dm.cameras[0]
    for name,camera,resolution,fov in [('RGB',cam.rgb_camera,[640,360],69.),('depth',cam.depth_camera,[848,480],87.)]:
        k=camera.get_intrinsics_matrix()
        k=k.detach().cpu().numpy() if hasattr(k,"detach") else np.asarray(k)
        actual=np.degrees(2*np.arctan(resolution[0]/(2*k[0,0])))
        check('D435 '+name+' horizontal FOV',abs(actual-fov)<.1,hfov_deg=float(actual),expected_deg=fov,intrinsics=k.tolist())
    image=np.asarray(Image.open(output/validator.snapshots['color_image']))
    check('D435 separate RGB resolution',image.shape== (360,640,3),shape=list(image.shape))
    depth=np.load(output/validator.snapshots['depth_image'])
    finite=depth[np.isfinite(depth)]
    check('D435 native depth resolution and range',depth.shape==(480,848) and finite.min()>=.2 and finite.max()<=10.,shape=list(depth.shape),min_m=float(finite.min()),max_m=float(finite.max()))
    pts=np.load(output/validator.snapshots['point_cloud'])
    radius=np.linalg.norm(pts,axis=1)
    elevation=np.degrees(np.arctan2(pts[:,2],np.linalg.norm(pts[:,:2],axis=1)))
    az=np.arctan2(pts[:,1],pts[:,0]);sectors=np.unique(((az+np.pi)/(2*np.pi)*12).astype(int)%12)
    check('MID360 actual point geometry',len(pts)>100 and radius.min()>=.099 and radius.max()<=70.01 and elevation.min()>=-7.2 and elevation.max()<=52.2,points=len(pts),range_m=[float(radius.min()),float(radius.max())],elevation_deg=[float(elevation.min()),float(elevation.max())],azimuth_sectors_of_12=len(sectors))
    a0,e0=mid360_directions(0);a1,e1=mid360_directions(1)
    check('MID360 procedural changing scan pattern',len(a0)==20000 and not np.allclose(a0,a1) and not np.allclose(e0,e1),rays_per_scan=len(a0),configured_rays_per_sim_second=200000,limitation=PROFILE['lidar']['model_limit'])
    imus=[s for s in dm.imu_history if s['robot']==0]
    imu=np.array([[s['time'],*s['linear_acceleration'],*s['angular_velocity']] for s in imus])
    np.save(output/'imu.npy',imu)
    if len(imu)>100:
        dt=np.diff(imu[:,0]);hz=(len(imu)-1)/(imu[-1,0]-imu[0,0])
        check('MID360 IMU physics sampling 200Hz',np.isfinite(imu).all() and abs(hz-200)<1 and np.max(dt)<.0101,samples=len(imu),simulated_hz=float(hz),max_gap_s=float(dt.max()))
        early=imu[(imu[:,0]>imu[0,0]+.4)&(imu[:,0]<imu[0,0]+1.5)]
        gravity=float(np.median(np.linalg.norm(early[:,1:4],axis=1))) if len(early) else 0.
        check('MID360 IMU stationary specific force includes gravity',8.5<gravity<11.,median_acceleration_m_s2=gravity)
        gyro=float(np.max(np.linalg.norm(imu[:,4:7],axis=1)))
        check('MID360 IMU responds to motion',gyro>.2,maximum_gyro_rad_s=gyro)
    else:
        check('MID360 IMU physics sampling 200Hz',False,samples=len(imu))
    refs=np.asarray(getattr(validator,'imu_references',[]))
    if len(refs):
        error=float(np.sqrt(np.mean((refs[:,0:3]-refs[:,3:6])**2)))
        check('MID360 IMU gyro agrees with independent robot body state',error<.4,rmse_rad_s=error)
    check('MID360 IMU mounted with LiDAR',len(dm.sensor_manager.imus)==validator.env.unwrapped.scene.num_envs,extrinsics_status=PROFILE['extrinsics_status'],assumed_imu_to_lidar=PROFILE['imu']['position_in_lidar'])
    for name,times,expected in [('D435',dm.camera_history,30),('MID360 LiDAR',dm.lidar_history,10)]:
        times=np.asarray(times)
        measured=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
        check(name+' simulation-time output rate',abs(measured-expected)<1.1,configured_hz=expected,measured_simulated_hz=float(measured),frames=len(times))
    (output/'sensor_profile.json').write_text(json.dumps(PROFILE,indent=2))
    return checks
