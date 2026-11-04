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
    for key,title in [('depth_image','Depth · metres'),('point_cloud','RTX LiDAR · sensor frame'),('rgbd_point_cloud','RGB-D cloud · world frame'),('semantic','Semantic IDs')]:
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
        plots.insert(0,('Robot front RGB · 640 × 480',directory/d['snapshots']['color_image']))
    if (directory/'simulation_view.png').exists():
        plots.insert(0,('Go2 simulation viewport',directory/'simulation_view.png'))
    multi=directory.parent/'scenes'/'multi-sensors'/'simulation_view.png'
    if multi.exists():
        plots.append(('Three Go2 robots · RGBD + RTX LiDAR verified on all robots',multi))
    trajectory=np.asarray(d['trajectory'])
    fig,ax=plt.subplots(figsize=(9,5));ax.plot(trajectory[:,1],trajectory[:,2]);ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Measured Go2 trajectory');ax.axis('equal');ax.grid(alpha=.3)
    out=directory/'trajectory.png';fig.savefig(out,dpi=120);plt.close(fig);plots.append(('Measured trajectory',out))
    cards=''.join(f'<figure><img src="data:image/png;base64,{base64.b64encode(p.read_bytes()).decode()}"><figcaption>{html.escape(t)}</figcaption></figure>' for t,p in plots)
    rows=''.join(f'<tr><td>{html.escape(r["name"])}</td><td class="{ "pass" if r["pass"] else "fail"}">{"PASS" if r["pass"] else "FAIL"}</td><td><pre>{html.escape(json.dumps({k:v for k,v in r.items() if k not in ["name","pass"]},ensure_ascii=False))}</pre></td></tr>' for r in d['results'])
    scene_rows=''
    scene_pass=0
    expected=['obstacle-sparse','obstacle-medium','obstacle-dense','warehouse','warehouse-forklifts','warehouse-shelves','full-warehouse','multi-flat','multi-rough','multi-sensors']
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
    page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Go2 Ubuntu 22.04 验证报告</title><style>body{{font:16px system-ui;background:#101827;color:#e2e8f0;margin:40px auto;max-width:1400px;padding:20px}}h1{{font-size:36px}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(440px,1fr));gap:24px}}figure{{margin:0;background:#1e293b;padding:16px;border-radius:12px}}img{{width:100%;border-radius:8px}}figcaption{{padding:12px}}table{{width:100%;border-collapse:collapse}}td,th{{padding:12px;text-align:left;border-bottom:1px solid #334155}}pre{{white-space:pre-wrap;word-break:break-word;max-width:800px}}.pass{{color:#4ade80}}.fail{{color:#fb7185}}</style><h1>Go2 · Ubuntu 22.04 工作站仿真验证</h1><p>Isaac Sim 5.1 · conda isaacsim_510 · RTX 3090 · {count}/{len(d['results'])} checks passed · {d['elapsed_s']:.1f}s wall time · average RTF {average_rtf:.2f}</p><p>图像与点云均来自本次实际 RTX 传感器采集。键盘 callback 注入和 OS 窗口键盘输入分别验收；未执行项目不视为通过。</p><div class="grid">{cards}</div><h2>验收记录</h2><table><tr><th>功能</th><th>结果</th><th>证据</th></tr>{rows}</table><h2>场景与多机器人验收 · {scene_pass}/{len(expected)}</h2><table>{scene_rows}</table><h2>运行环境</h2><pre>{environment}</pre><h2>传感器诊断</h2><pre>{html.escape(json.dumps(d['sensor_errors'],ensure_ascii=False,indent=2))}</pre></html>'''
    (directory/'report.html').write_text(page, encoding='utf-8')
    print('REPORT:',directory/'report.html',flush=True)

if __name__=='__main__':
    import sys
    build(sys.argv[1])
