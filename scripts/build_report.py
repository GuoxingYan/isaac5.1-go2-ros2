"""Standalone evidence report, renders real captured arrays only."""
import base64
import html
import json
from pathlib import Path
import numpy as np

def build(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory)
    d=json.loads((directory/'results.json').read_text())
    plots=[]
    for key,title in [('depth_image','Depth · metres'),('point_cloud','MID360 · sensor frame'),('rgbd_point_cloud','RGB-D cloud · world frame'),('semantic','Semantic IDs')]:
        name=d['snapshots'].get(key)
        if not name:
            continue
        arr=np.load(directory/name)
        fig=plt.figure(figsize=(9,6))
        if 'cloud' in key:
            ax=fig.add_subplot(111,projection='3d')
            pts=arr.reshape(-1,3)
            pts=pts[np.isfinite(pts).all(1)]
            pts=pts[np.linalg.norm(pts,axis=1)<60][::max(1,len(pts)//15000)]
            ax.scatter(*pts.T,c=pts[:,2],s=.6,cmap='turbo')
            ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)')
            ax.view_init(elev=35,azim=-60)
        else:
            ax=fig.add_subplot(111)
            a=arr.copy()
            if a.ndim==3: a=a[:,:,0]
            if key=='depth_image': a=np.where(np.isfinite(a)&(a<50),a,np.nan)
            im=ax.imshow(a,cmap='turbo',vmin=0,vmax=20 if key=='depth_image' else None)
            fig.colorbar(im,ax=ax,label='metres' if key=='depth_image' else 'class ID')
        ax.set_title(title)
        out=directory/(key+'.png')
        fig.tight_layout();fig.savefig(out,dpi=120);plt.close(fig)
        plots.append((title,out))
    if 'color_image' in d['snapshots']:
        plots.insert(0,('D435 RGB · separate color optical frame',directory/d['snapshots']['color_image']))
    if (directory/'simulation_view.png').exists():
        plots.insert(0,('Go2 simulation viewport',directory/'simulation_view.png'))
    if (directory/'gui_gaussian_window.png').exists():
        plots.append(('Actual Isaac GUI with live D435 Gaussian RGB window',directory/'gui_gaussian_window.png'))
    multi=directory.parent/'scenes'/'multi-sensors'/'simulation_view.png'
    if multi.exists() and not d.get("sensor_profile"):
        plots.append(('Three Go2 robots · RGBD + RTX LiDAR verified on all robots',multi))
    if (directory/'imu.npy').exists():
        imu=np.load(directory/'imu.npy')
        fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
        for i,label in enumerate(['X','Y','Z']):
            axes[0].plot(imu[:,0],imu[:,i+1],label=label,linewidth=.5)
            axes[1].plot(imu[:,0],imu[:,i+4],label=label,linewidth=.5)
        axes[0].set(ylabel='specific force (m/s²)',title='MID360 integrated IMU · physics samples, simulated time')
        axes[1].set(xlabel='simulation time (s)',ylabel='gyro (rad/s)')
        for ax in axes: ax.legend();ax.grid(alpha=.2)
        out=directory/'imu.png';fig.tight_layout();fig.savefig(out,dpi=120);plt.close(fig)
        plots.append(('MID360 IMU · gravity + motion response',out))
    spawn_section=''
    scene_path=directory/'scene_evidence.json'
    if scene_path.exists():
        scene=json.loads(scene_path.read_text())
        from env.interiorgs_scene import InteriorGSScene
        layout=InteriorGSScene('data',scene['spawn']['seed'],scene['spawn']['required_clearance_m'])
        track=np.asarray(d['trajectory'])
        xy=np.array([layout.world_to_pixel(s[1],s[2]) for s in track])
        fig,ax=plt.subplots(figsize=(7,9))
        ax.imshow(layout.image,cmap='gray',origin='lower',vmin=0,vmax=255)
        ax.plot(xy[:,1],xy[:,0],color='tab:red',linewidth=1.5,label='measured robot trajectory')
        r,c=scene['spawn']['pixel_row_col'];ax.scatter(c,r,marker='*',s=120,color='tab:blue',label='occupancy-sampled spawn')
        ax.set(xlabel='map column (X flipped)',ylabel='map row (+world Y)',title='Occupancy + mesh clearance + actual Go2 trajectory');ax.legend()
        out=directory/'occupancy_trajectory.png';fig.tight_layout();fig.savefig(out,dpi=120);plt.close(fig)
        plots.append(('Occupancy random spawn and measured path · 0.05 m/pixel',out))
        for key in ['W','S','A','D','Z','C']:
            path=directory/f'movement_{key}_rgb.png'
            if path.exists(): plots.append((f'{key} · actual USDZ Gaussian RGB after keyboard movement',path))
    if scene_path.exists():
        sweep=directory/'spawn_sweep.json'
        if sweep.exists():
            summary=json.loads(sweep.read_text())
            summary.pop('samples',None)
            spawn_section='<h2>随机出生补充核验</h2><pre>'+html.escape(json.dumps(summary,ensure_ascii=False,indent=2))+'</pre>'
        for seed in [7,123]:
            folder=directory.parent/f'interiorgs-seed{seed}'
            smoke=folder/'smoke.json'
            if smoke.exists():
                evidence=json.loads(smoke.read_text())
                spawn_section+=f'<h3>Seed {seed} · {"PASS" if evidence["pass"] else "FAIL"}</h3><pre>'+html.escape(json.dumps(evidence,ensure_ascii=False,indent=2))+'</pre>'
                viewport=folder/'simulation_view.png'
                if viewport.exists(): plots.insert(0,(f'Seed {seed} · Go2 on actual collision mesh',viewport))
                picture=folder/'scene_rgb.png'
                if picture.exists(): plots.append((f'Seed {seed} · independent physical spawn Gaussian RGB',picture))
    trajectory=np.asarray(d['trajectory'])
    fig,ax=plt.subplots(figsize=(9,5));ax.plot(trajectory[:,1],trajectory[:,2]);ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Measured Go2 trajectory');ax.axis('equal');ax.grid(alpha=.3)
    out=directory/'trajectory.png';fig.savefig(out,dpi=120);plt.close(fig);plots.append(('Measured trajectory',out))
    cards=''.join(f'<figure><img src="data:image/png;base64,{base64.b64encode(p.read_bytes()).decode()}"><figcaption>{html.escape(t)}</figcaption></figure>' for t,p in plots)
    rows=''.join(f'<tr><td>{html.escape(r["name"])}</td><td class="{ "pass" if r["pass"] else "fail"}">{"PASS" if r["pass"] else "FAIL"}</td><td><pre>{html.escape(json.dumps({k:v for k,v in r.items() if k not in ["name","pass"]},ensure_ascii=False))}</pre></td></tr>' for r in d['results'])
    scene_rows=''
    scene_pass=0
    expected=['obstacle-sparse','obstacle-medium','obstacle-dense','warehouse','warehouse-forklifts','warehouse-shelves','full-warehouse','multi-flat','multi-rough','multi-sensors']
    if d.get('sensor_profile'):
        expected=[]
    for name in expected:
        path=directory.parent/'scenes'/name/'smoke.json'
        if not path.exists():
            scene_rows+=f'<tr><td>{name}</td><td class="fail">NOT VERIFIED</td><td>No completed evidence</td></tr>'
            continue
        scene=json.loads(path.read_text())
        scene_pass+=bool(scene['pass'])
        details=json.dumps(scene.get('control_checks',[]),ensure_ascii=False)
        scene_rows+=f'<tr><td>{html.escape(name)}</td><td class="{ "pass" if scene["pass"] else "fail"}">{"PASS" if scene["pass"] else "FAIL"}</td><td>{scene["num_envs"]} robot(s), {scene["policy"]}, {scene["steps"]} steps, warehouse meshes={scene["warehouse_meshes"]}<pre>{html.escape(details)}</pre></td></tr>'
    environment_path=directory.parent/'environment.json'
    environment=html.escape(environment_path.read_text(encoding='utf-8')) if environment_path.exists() else ''
    sim_seconds=max(0,len(d['trajectory'])-1)*.02
    average_rtf=sim_seconds/max(d['elapsed_s'],.001)
    count=sum(r['pass'] for r in d['results'])
    hardware=''
    if d.get('sensor_profile'):
        profile=d['sensor_profile']
        hardware='<h2>Our Go2 · D435 / MID360 / MID360 IMU</h2><p>外参沿用原安装位置，尚未实机标定。D435 是独立彩色/深度视场的理想几何模型；MID360 使用真实场景三角形求交和程序化扫描覆盖近似，非厂商专有扫描轨迹；IMU 为同位置物理采样，无硬件噪声标定，不发布虚构姿态。主分支使用通用相机与 Example_Rotary，未配置独立 IMU。</p><pre>'+html.escape(json.dumps(profile,ensure_ascii=False,indent=2))+'</pre>'
    if scene_path.exists():
        hardware+='<h2>InteriorGS · RGB / metric geometry separation</h2><p>RGB直接解码839873.usdz内NuRec高斯及SH3系数并用gsplat渲染；mesh供CUDA三角形射线深度/LiDAR及PhysX碰撞使用，无高斯几何或合成地板混入metric场景。所有安装位姿与地图坐标使用同一米制Z-up原始坐标系。高斯图像不叠加机器人mesh；当前mesh深度/LiDAR亦未包含机器人自身遮挡。未建模硬件噪声、反射率与扫描运动畸变，采样率使用模拟时间。</p><pre>'+html.escape(scene_path.read_text())+'</pre>'
    acquisition='RGB来自USDZ内真实高斯的gsplat渲染；深度与LiDAR来自碰撞mesh的Warp CUDA射线求交，物理碰撞由PhysX处理。' if scene_path.exists() else '图像与点云来自本次实际RTX传感器采集。'
    notes=directory/'VALIDATED.md'
    validation_notes='<h2>验收范围与限制</h2><pre>'+html.escape(notes.read_text())+'</pre>' if notes.exists() else ''
    scene_section=f'<h2>场景与多机器人验收 · {scene_pass}/{len(expected)}</h2><table>{scene_rows}</table>' if expected else ''
    page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Go2 Ubuntu 22.04 验证报告</title><style>body{{font:16px system-ui;background:#101827;color:#e2e8f0;margin:40px auto;max-width:1400px;padding:20px}}h1{{font-size:36px}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(440px,1fr));gap:24px}}figure{{margin:0;background:#1e293b;padding:16px;border-radius:12px}}img{{width:100%;border-radius:8px}}figcaption{{padding:12px}}table{{width:100%;border-collapse:collapse}}td,th{{padding:12px;text-align:left;border-bottom:1px solid #334155}}pre{{white-space:pre-wrap;word-break:break-word;max-width:800px}}.pass{{color:#4ade80}}.fail{{color:#fb7185}}</style><h1>Go2 · Ubuntu 22.04 工作站仿真验证</h1><p>Isaac Sim 5.1 · conda isaacsim_510 · RTX 3090 · {count}/{len(d['results'])} checks passed · {d['elapsed_s']:.1f}s wall time · average RTF {average_rtf:.2f}</p><p>{acquisition}键盘 callback 注入和 OS 窗口键盘输入分别验收；未执行项目不视为通过。</p><div class="grid">{cards}</div>{hardware}<h2>验收记录</h2><table><tr><th>功能</th><th>结果</th><th>证据</th></tr>{rows}</table>{scene_section}{spawn_section}{validation_notes}<h2>运行环境</h2><pre>{environment}</pre><h2>传感器诊断</h2><pre>{html.escape(json.dumps(d['sensor_errors'],ensure_ascii=False,indent=2))}</pre></html>'''
    (directory/'report.html').write_text(page, encoding='utf-8')
    print('REPORT:',directory/'report.html',flush=True)

if __name__=='__main__':
    import sys
    build(sys.argv[1])
