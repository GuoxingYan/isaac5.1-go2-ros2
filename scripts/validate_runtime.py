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
        self.finished=False
        self.keys=['W','S','A','D','Z','C']
        self.start=None
        self.started=time.time()
        self.snapshots={}
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
        i,t=divmod(step-100,150)
        if i>=len(self.keys):
            if step==1000:
                self.ros_start=self.state()
                self.ros_received=False
            if 1000 <= step < 1100:
                request=Path('outputs/ros_command_request.json')
                temporary=request.with_suffix('.tmp')
                temporary.write_text(json.dumps({'timestamp':time.time()}))
                temporary.replace(request)
                self.ros_received = self.ros_received or bool(ctrl.base_vel_cmd_input[0,0]>.4)
            if step==1150:
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
        if t==10:
            self.command=ctrl.base_vel_cmd_input[0].tolist()
        if t==100:
            end=self.state()
            body=Rotation.from_quat(self.start[[4,5,6,3]]).inv().apply(end[:3]-self.start[:3])
            yaw0=Rotation.from_quat(self.start[[4,5,6,3]]).as_euler('xyz')[2]
            yaw1=Rotation.from_quat(end[[4,5,6,3]]).as_euler('xyz')[2]
            yaw=float(np.arctan2(np.sin(yaw1-yaw0),np.cos(yaw1-yaw0)))
            axis,sign={'W':(0,1),'S':(0,-1),'A':(1,1),'D':(1,-1),'Z':(2,1),'C':(2,-1)}[self.keys[i]]
            amount=float(body[axis] if axis<2 else yaw)
            expected={'W':[1.,0.,0.],'S':[-1.,0.,0.],'A':[0.,1.,0.],'D':[0.,-1.,0.],'Z':[0.,0.,1.],'C':[0.,0.,-1.]}[self.keys[i]]
            passed=amount*sign>(0.1 if axis<2 else .15) and end[2]>.2 and np.allclose(self.command,expected)
            self.results.append({'name':self.keys[i]+' keyboard callback → physics','pass':bool(passed),
                'command':self.command,'body_displacement_m':body.tolist(),'yaw_change_rad':yaw,'height_m':float(end[2]),
                'input_method':'X11 XTest → window → Carb callback' if self.keyboard else 'Carb callback injection (not OS key delivery)'})
            self.event(self.keys[i],False)
        if t==110:
            self.results.append({'name':self.keys[i]+' release stop command','pass':bool((ctrl.base_vel_cmd_input==0).all())})
            print('\n[VALIDATION]',self.results[-2],flush=True)

    def after_step(self,step):
        if step == 60 and self.keyboard:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            self.viewport_capture = capture_viewport_to_file(get_active_viewport(), str(self.output/'simulation_view.png'))
        state=self.state()
        self.trajectory.append([step,*state.tolist()])
        d=self.dm.latest
        if step>=60 and d.get('color_image') and d.get('depth_image'):
            candidate=np.load(DATA_DIR/d['depth_image'])
            valid=candidate[np.isfinite(candidate)&(candidate>0)&(candidate<100)]
            if valid.size<100 or np.ptp(valid)<.1:
                return
            for key in ['color_image','depth_image','point_cloud','rgbd_point_cloud','semantic']:
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
            (self.output/'sensor_metadata.json').write_text(json.dumps(d,indent=2))

    def finish(self):
        if self.keyboard:
            for key in self.keys:
                self.keyboard.event(key,False)
        self.results.append({'name':'physics finite and robot upright','pass':bool(np.isfinite(self.trajectory).all() and min(s[3] for s in self.trajectory[min(100,len(self.trajectory)-1):])>.2)})
        for key in ['color_image','depth_image','point_cloud','rgbd_point_cloud','semantic']:
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
                    detail.update(valid_pixels=int(valid.size),unit='metres',source='RTX distance_to_image_plane')
            self.results.append({'name':key,'pass':passed,**detail})
        topics=self.output/'ros_topics.json'
        if topics.exists():
            evidence=json.loads(topics.read_text())
            for key,count in evidence['counts'].items():
                self.results.append({'name':'ROS2 '+key+' received','pass':count>0,'frames':count,**evidence['details'].get(key,{})})
        if self.test_keyboard:
            self.results.append({'name':'keyboard event delivery', 'pass':bool(len(ctrl.keyboard_events)>10), 'events':len(ctrl.keyboard_events), 'event_tail':list(ctrl.keyboard_events)[-20:], 'method':'X11 XTest' if self.keyboard else 'callback injection'})
        result={'results':self.results,'snapshots':self.snapshots,'elapsed_s':time.time()-self.started,
                'trajectory':self.trajectory,'sensor_errors':self.dm.errors}
        (self.output/'results.json').write_text(json.dumps(result,indent=2))
        from scripts.build_report import build
        build(self.output)
