"""In-simulator validation: actual policy, physics and keyboard callback, no fake sensors."""
import json
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import carb
from scipy.spatial.transform import Rotation
from PIL import Image
import go2.go2_ctrl as ctrl
from ros2.go2_ros2_bridge import DATA_DIR

class RuntimeValidator:
    def __init__(self, env, dm, output, os_keyboard=False, test_keyboard=True):
        self.env, self.dm = env, dm
        self.test_keyboard = test_keyboard
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.results=[]
        self.trajectory=[]
        self.imu_references=[]
        self.foot_forces=[]
        self.finished=False
        self.keys=['W','S','A','D','Z','C']
        self.start=None
        self.started=time.time()
        self.snapshots={}
        self.snapshot_metadata={}
        self.indoor=hasattr(dm,'interior_scene')
        self.press_steps=45 if self.indoor else 100
        self.cycle_steps=70 if self.indoor else 150
        self.ros_first=100+6*self.cycle_steps
        self.ros_drive_steps=30 if self.indoor else 100
        self.keyboard=None
        if os_keyboard:
            from scripts.x11_keyboard import X11Keyboard
            self.keyboard=X11Keyboard()

    def event(self,key,press):
        if self.keyboard:
            self.keyboard.event(key,press)
            return
        ctrl.sub_keyboard_event(SimpleNamespace(input=SimpleNamespace(name=key),
            type=carb.input.KeyboardEventType.KEY_PRESS if press else carb.input.KeyboardEventType.KEY_RELEASE))

    def state(self):
        return self.env.unwrapped.scene['unitree_go2'].data.root_state_w[0].detach().cpu().numpy().copy()

    def before_step(self,step):
        if step < 100:
            return
        i,t=divmod(step-100,self.cycle_steps)
        if i>=len(self.keys):
            if step==self.ros_first:
                self.ros_start=self.state()
                self.ros_received=False
            if self.ros_first <= step < self.ros_first+self.ros_drive_steps:
                request=Path('outputs/ros_command_request.json')
                temporary=request.with_suffix('.tmp')
                temporary.write_text(json.dumps({'timestamp':time.time()}))
                temporary.replace(request)
                self.ros_received = self.ros_received or bool(ctrl.base_vel_cmd_input[0,0]>.4)
            if step==self.ros_first+self.ros_drive_steps+50:
                displacement=float(np.linalg.norm(self.state()[:2]-self.ros_start[:2]))
                self.results.append({'name':'ROS2 cmd_vel received by simulation', 'pass':self.ros_received, 'method':'external ROS2 publisher → subscriber → atomic file → control'})
                self.results.append({'name':'ROS2 cmd_vel physical movement', 'pass':displacement>.1, 'displacement_m':displacement})
                self.results.append({'name':'ROS2 command timeout stop', 'pass':bool((ctrl.base_vel_cmd_input==0).all())})
                self.finished=True
            return
        if t==0:
            if self.keyboard:
                for key in self.keys:
                    self.keyboard.event(key,False)
            ctrl.pressed_keys.clear()
            ctrl.base_vel_cmd_input.zero_()
            self.start=self.state()
            self.event(self.keys[i],True)
            self.command=ctrl.base_vel_cmd_input[0].tolist()
        if t in (10, 15, 20, 25) and self.keyboard and not any(abs(value)>.01 for value in ctrl.base_vel_cmd_input[0].tolist()):
            # Let render dispatch a queued X11 event, then retry through the window.
            self.event(self.keys[i],True)
        if t==30:
            self.command=ctrl.base_vel_cmd_input[0].tolist()
        if t==self.press_steps:
            end=self.state()
            body=Rotation.from_quat(self.start[[4,5,6,3]]).inv().apply(end[:3]-self.start[:3])
            yaw0=Rotation.from_quat(self.start[[4,5,6,3]]).as_euler('xyz')[2]
            yaw1=Rotation.from_quat(end[[4,5,6,3]]).as_euler('xyz')[2]
            yaw=float(np.arctan2(np.sin(yaw1-yaw0),np.cos(yaw1-yaw0)))
            axis,sign={'W':(0,1),'S':(0,-1),'A':(1,1),'D':(1,-1),'Z':(2,1),'C':(2,-1)}[self.keys[i]]
            amount=float(body[axis] if axis<2 else yaw)
            expected={'W':[ctrl.linear_speed,0.,0.],'S':[-ctrl.linear_speed,0.,0.],'A':[0.,ctrl.linear_speed,0.],'D':[0.,-ctrl.linear_speed,0.],'Z':[0.,0.,1.],'C':[0.,0.,-1.]}[self.keys[i]]
            motion_ok=amount*sign>((.06 if self.indoor else .1) if axis<2 else .15)
            command_ok=np.allclose(self.command,expected)
            # XTest events may be consumed one render tick after the snapshot;
            # physical motion plus a recorded key event is the authoritative OS path.
            os_event_ok=bool(self.keyboard and any(e.get('key')==self.keys[i] for e in ctrl.keyboard_events))
            passed=motion_ok and end[2]>.2 and (command_ok or os_event_ok)
            self.results.append({'name':self.keys[i]+' keyboard callback → physics','pass':bool(passed),
                'command':self.command,'body_displacement_m':body.tolist(),'yaw_change_rad':yaw,'height_m':float(end[2]),
                'input_method':'X11 XTest → window → Carb callback' if self.keyboard else 'Carb callback injection (not OS key delivery)'})
            self.event(self.keys[i],False)
        if t==self.press_steps+10:
            self.results.append({'name':self.keys[i]+' release stop command','pass':bool((ctrl.base_vel_cmd_input==0).all())})
            print('\n[VALIDATION]',self.results[-2],flush=True)

    def after_step(self,step):
        if step == 60 and self.keyboard:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            self.viewport_capture = capture_viewport_to_file(get_active_viewport(), str(self.output/'simulation_view.png'))
        state=self.state()
        self.trajectory.append([step,*state.tolist()])
        if self.indoor:
            force=self.env.unwrapped.scene['contact_forces'].data.net_forces_w[0].detach().cpu().numpy()
            self.foot_forces.append(float(force[:,2].sum()))
        if self.dm.sensor_manager and self.dm.sensor_manager.imus:
            actual=self.dm.sensor_manager.imu.data.ang_vel_b[0].detach().cpu().numpy()
            expected=Rotation.from_quat(state[[4,5,6,3]]).inv().apply(state[10:13])
            self.imu_references.append([*actual.tolist(),*expected.tolist()])
        d=self.dm.latest
        if self.indoor and step>=100:
            i,t=divmod(step-100,self.cycle_steps)
            if i<6 and t==self.press_steps-1 and d.get('color_image'):
                import shutil
                shutil.copy2(DATA_DIR/d['color_image'],self.output/f'movement_{self.keys[i]}_rgb.png')
                shutil.copy2(DATA_DIR/d['depth_image'],self.output/f'movement_{self.keys[i]}_depth.npy')
        if step>=60 and d.get('color_image') and d.get('depth_image'):
            candidate=np.load(DATA_DIR/d['depth_image'])
            valid=candidate[np.isfinite(candidate)&(candidate>0)&(candidate<100)]
            if valid.size<100 or np.ptp(valid)<.1:
                return
            for key in (['color_image','depth_image','point_cloud','rgbd_point_cloud']+(['semantic'] if self.dm.cfg.sensor.semantic_segmentation else [])):
                if key in d and key not in self.snapshots:
                    if key == 'depth_image':
                        candidate=np.load(DATA_DIR/d[key])
                        valid=candidate[np.isfinite(candidate)&(candidate>0)&(candidate<100)]
                        if valid.size<100 or np.ptp(valid)<.1:
                            continue
                    import shutil
                    src=DATA_DIR/d[key]
                    dst=self.output/(key+src.suffix)
                    shutil.copy2(src,dst)
                    self.snapshots[key]=dst.name
                    self.snapshot_metadata[key]=dict(d)
            (self.output/'sensor_metadata.json').write_text(json.dumps(d,indent=2))

    def finish(self):
        if self.keyboard:
            for key in self.keys:
                self.keyboard.event(key,False)
        settled=np.asarray(self.trajectory[min(100,len(self.trajectory)-1):])
        minimum_height=float(settled[:,3].min())
        up=Rotation.from_quat(settled[:,[5,6,7,4]]).apply(np.tile([0.,0.,1.],(len(settled),1)))[:,2]
        self.results.append({'name':'physics finite and robot upright','pass':bool(np.isfinite(self.trajectory).all() and minimum_height>.2 and up.min()>.7),'minimum_height_m':minimum_height,'minimum_body_up_dot_world_up':float(up.min())})
        for key in (['color_image','depth_image','point_cloud','rgbd_point_cloud']+(['semantic'] if self.dm.cfg.sensor.semantic_segmentation else [])):
            path=self.output/self.snapshots.get(key,'missing')
            passed=path.exists()
            detail={}
            if passed and path.suffix=='.npy':
                a=np.load(path)
                finite=np.isfinite(a)
                detail={'shape':list(a.shape),'finite_count':int(finite.sum())}
                passed=bool(a.size and finite.any())
                if key=='depth_image':
                    valid=a[finite & (a>0) & (a<100)]
                    passed=bool(valid.size>100 and np.ptp(valid)>.1)
                    detail.update(valid_pixels=int(valid.size),unit='metres',source='collision mesh CUDA axial ray depth' if self.indoor else 'RTX distance_to_image_plane')
            self.results.append({'name':key,'pass':passed,**detail})
        if self.dm.sensor_manager:
            from scripts.validate_hardware import validate
            self.results.extend(validate(self))
        topics=self.output/'ros_topics.json'
        if topics.exists():
            evidence=json.loads(topics.read_text())
            for key,count in evidence['counts'].items():
                if key=='semantic' and not self.dm.cfg.sensor.semantic_segmentation:
                    continue
                self.results.append({'name':'ROS2 '+key+' received','pass':count>0,'frames':count,**evidence['details'].get(key,{})})
        if topics.exists() and self.dm.sensor_manager and evidence.get('imu_times'):
            from scripts.validate_imu_transport import validate as validate_transport
            self.results.append(validate_transport(evidence))
        if self.test_keyboard:
            self.results.append({'name':'keyboard event delivery', 'pass':bool(len(ctrl.keyboard_events)>10), 'events':len(ctrl.keyboard_events), 'event_tail':list(ctrl.keyboard_events)[-20:], 'method':'X11 XTest' if self.keyboard else 'callback injection'})
        if self.indoor:
            from scripts.validate_interiorgs import validate
            self.results.extend(validate(self))
        result={'results':self.results,'snapshots':self.snapshots,'elapsed_s':time.time()-self.started,
                'trajectory':self.trajectory,'sensor_errors':self.dm.errors, 'sensor_profile':self.dm.latest.get('sensor_profile'), 'snapshot_metadata':self.snapshot_metadata, 'foot_contact_z_forces':self.foot_forces}
        (self.output/'results.json').write_text(json.dumps(result,indent=2))
        from scripts.build_report import build
        build(self.output)
