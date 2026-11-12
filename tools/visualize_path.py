#!/usr/bin/env python3
"""在浏览器中交互式查看 JSON 位姿路径。仅依赖 Python 标准库。"""

from __future__ import annotations

import argparse
import json
import math
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


DEFAULT_PATH = "/data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2/data/path_occupancy.json"

PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>路径查看器</title>
<style>
:root{color-scheme:light;--ink:#1e292b;--muted:#738082;--line:#dce4e2;--paper:#f7faf8;--green:#147d68;--orange:#e27a39}
*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
header{height:58px;padding:0 22px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid var(--line);background:#fff}
h1{font-size:16px;margin:0;font-weight:650}header span{font-size:12px;color:var(--muted)}main{height:calc(100vh - 58px);min-height:460px;display:grid;grid-template-columns:minmax(0,1fr) 286px}
.stage{position:relative;min-width:0;overflow:hidden;background-color:#f7faf8;background-image:linear-gradient(#e8efec 1px,transparent 1px),linear-gradient(90deg,#e8efec 1px,transparent 1px);background-size:32px 32px}
canvas{width:100%;height:100%;display:block;touch-action:none}.toolbar{position:absolute;top:14px;left:14px;display:flex;gap:5px;padding:5px;background:#ffffffed;border:1px solid var(--line);border-radius:7px;box-shadow:0 3px 12px #20302b12}
button,.file-label{font:inherit;color:var(--ink);background:white;border:1px solid var(--line);border-radius:5px;padding:6px 9px;cursor:pointer}button:hover,.file-label:hover{border-color:#98aaa5;background:#f4f8f6}button:disabled{opacity:.45;cursor:default}.toolbar button{width:34px;padding:5px;font-size:16px}.file-label{font-size:12px}#file{display:none}
aside{background:#fff;border-left:1px solid var(--line);padding:19px 17px;overflow:auto}.section{padding-bottom:17px;margin-bottom:17px;border-bottom:1px solid var(--line)}.eyebrow{font-size:10px;font-weight:700;letter-spacing:.08em;color:var(--muted);text-transform:uppercase;margin-bottom:8px}.filename{font-size:14px;font-weight:600;overflow-wrap:anywhere}.meta{margin-top:4px;font-size:12px;color:var(--muted)}
.field{display:flex;align-items:center;justify-content:space-between;gap:10px;margin:12px 0}.field label{font-size:12px;color:#465355}.field input[type=range]{width:115px;accent-color:var(--green)}.value{width:34px;text-align:right;font-size:12px;font-variant-numeric:tabular-nums}.playrow{display:flex;gap:7px;align-items:center}.playrow button{color:white;background:var(--green);border-color:var(--green);font-weight:600}.playrow button:hover{background:#106b5a}.playrow input{flex:1;accent-color:var(--green)}
.stat{display:grid;grid-template-columns:1fr auto;gap:7px;font-size:12px;margin:7px 0}.stat span:first-child{color:var(--muted)}.stat strong{font-weight:600;font-variant-numeric:tabular-nums}.hint{font-size:11px;color:var(--muted);line-height:1.7}.legend{display:flex;gap:12px;font-size:11px;color:var(--muted)}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px;background:var(--green)}.dot.end{background:var(--orange)}
@media(max-width:720px){main{height:auto;min-height:calc(100vh - 58px);grid-template-columns:1fr;grid-template-rows:minmax(52vh,1fr) auto}.stage{min-height:52vh}aside{border-left:0;border-top:1px solid var(--line);display:grid;grid-template-columns:1fr 1fr;column-gap:20px;padding:14px}.section{margin-bottom:10px;padding-bottom:10px}}
</style></head><body>
<header><h1>路径查看器 <span style="font-weight:400">/ 平面位姿</span></h1><span id="frame">frame: --</span></header>
<main><section class="stage"><canvas id="canvas"></canvas><div class="toolbar"><button id="fit" title="适应视图">⌖</button><button id="minus" title="缩小">−</button><button id="plus" title="放大">+</button><label class="file-label" for="file">打开 JSON</label><input id="file" type="file" accept=".json,application/json"></div></section>
<aside><section class="section"><div class="eyebrow">轨迹文件</div><div class="filename" id="name">加载中</div><div class="meta" id="count"></div><div class="legend" style="margin-top:12px"><span><i class="dot"></i>起点</span><span><i class="dot end"></i>终点</span></div></section>
<section class="section"><div class="eyebrow">显示</div><div class="field"><label for="density">箭头间隔</label><input id="density" type="range" min="1" max="20" value="1"><span class="value" id="densityVal">1</span></div><div class="field"><label for="length">箭头长度</label><input id="length" type="range" min="8" max="42" value="20"><span class="value" id="lengthVal">20</span></div><div class="hint">朝向箭头由位姿四元数计算。滚轮缩放，按住鼠标拖动平移。</div></section>
<section class="section"><div class="eyebrow">路径回放</div><div class="playrow"><button id="play">▶ 播放</button><input id="progress" type="range" min="0" max="0" value="0"><span class="value" id="progressVal">0</span></div><div class="field"><label for="speed">播放速度</label><input id="speed" type="range" min="1" max="10" value="4"><span class="value" id="speedVal">4x</span></div></section>
<section><div class="eyebrow">当前位姿</div><div class="stat"><span>点位</span><strong id="point">-</strong></div><div class="stat"><span>X</span><strong id="x">-</strong></div><div class="stat"><span>Y</span><strong id="y">-</strong></div><div class="stat"><span>航向</span><strong id="yaw">-</strong></div></section></aside></main>
<script>
const canvas=document.querySelector('#canvas'),ctx=canvas.getContext('2d');let poses=[],frame='map',fileName='',selected=0,view={scale:1,ox:0,oy:0},playing=false,timer=null,drag=null;
const $=s=>document.querySelector(s),fmt=n=>Number(n).toFixed(3);
function yawOf(p){return Math.atan2(2*(p.qw*p.qz+p.qx*p.qy),1-2*(p.qy*p.qy+p.qz*p.qz))}
function resize(){const r=canvas.getBoundingClientRect(),d=window.devicePixelRatio||1;canvas.width=Math.round(r.width*d);canvas.height=Math.round(r.height*d);ctx.setTransform(d,0,0,d,0,0);draw()}
function fit(){if(!poses.length)return;const r=canvas.getBoundingClientRect(),xs=poses.map(p=>p.x),ys=poses.map(p=>p.y),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),pad=55;view.scale=Math.min((r.width-pad*2)/Math.max(maxX-minX,.1),(r.height-pad*2)/Math.max(maxY-minY,.1));view.ox=(r.width-(minX+maxX)*view.scale)/2;view.oy=(r.height+(minY+maxY)*view.scale)/2;draw()}
function screen(p){return{x:view.ox+p.x*view.scale,y:view.oy-p.y*view.scale}}
function draw(){if(!canvas.width)return;const r=canvas.getBoundingClientRect();ctx.clearRect(0,0,r.width,r.height);if(!poses.length)return;ctx.lineCap='round';ctx.lineJoin='round';ctx.beginPath();poses.forEach((p,i)=>{const q=screen(p);i?ctx.lineTo(q.x,q.y):ctx.moveTo(q.x,q.y)});ctx.strokeStyle='#147d68';ctx.lineWidth=3;ctx.stroke();
const step=Number($('#density').value),len=Number($('#length').value);poses.forEach((p,i)=>{if(i%step&&i!==poses.length-1)return;const q=screen(p),a=-yawOf(p),l=len,head=5;ctx.save();ctx.translate(q.x,q.y);ctx.rotate(a);ctx.beginPath();ctx.moveTo(-l*.35,0);ctx.lineTo(l*.38,0);ctx.moveTo(l*.38,0);ctx.lineTo(l*.38-head,-head*.65);ctx.moveTo(l*.38,0);ctx.lineTo(l*.38-head,head*.65);ctx.strokeStyle='#253537';ctx.lineWidth=1.8;ctx.stroke();ctx.restore()});
const first=screen(poses[0]),last=screen(poses.at(-1));[[first,'#147d68'],[last,'#e27a39']].forEach(([q,c])=>{ctx.beginPath();ctx.arc(q.x,q.y,5,0,Math.PI*2);ctx.fillStyle=c;ctx.fill();ctx.lineWidth=2;ctx.strokeStyle='white';ctx.stroke()});const s=screen(poses[selected]);ctx.beginPath();ctx.arc(s.x,s.y,7,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.lineWidth=3;ctx.strokeStyle='#147d68';ctx.stroke()}
function setSelected(i){selected=Math.max(0,Math.min(poses.length-1,i));const p=poses[selected];$('#point').textContent=`${selected+1} / ${poses.length}`;$('#x').textContent=fmt(p.x)+' m';$('#y').textContent=fmt(p.y)+' m';$('#yaw').textContent=fmt(yawOf(p)*180/Math.PI)+'°';$('#progress').value=selected;$('#progressVal').textContent=selected+1;draw()}
function load(data,name){if(!data||!Array.isArray(data.poses)||!data.poses.length)throw Error('JSON 中没有非空 poses 数组');for(const [i,p] of data.poses.entries()){for(const k of ['x','y','qx','qy','qz','qw'])if(!Number.isFinite(Number(p[k])))throw Error(`第 ${i+1} 个位姿缺少有效字段 ${k}`)}poses=data.poses.map(p=>Object.fromEntries(['x','y','z','qx','qy','qz','qw'].map(k=>[k,Number(p[k]??0)])));frame=data.frame_id||'map';fileName=name;$('#name').textContent=name;$('#count').textContent=`${poses.length} 个位姿点`;$('#frame').textContent=`frame: ${frame}`;$('#progress').max=poses.length-1;setSelected(0);fit()}
$('#density').oninput=e=>{$('#densityVal').textContent=e.target.value;draw()};$('#length').oninput=e=>{$('#lengthVal').textContent=e.target.value;draw()};$('#speed').oninput=e=>$('#speedVal').textContent=e.target.value+'x';$('#progress').oninput=e=>setSelected(Number(e.target.value));$('#fit').onclick=fit;$('#plus').onclick=()=>zoom(1.2);$('#minus').onclick=()=>zoom(1/1.2);function zoom(f,x=canvas.clientWidth/2,y=canvas.clientHeight/2){view.ox=x-(x-view.ox)*f;view.oy=y-(y-view.oy)*f;view.scale*=f;draw()}
$('#play').onclick=()=>{playing=!playing;$('#play').textContent=playing?'❚❚ 暂停':'▶ 播放';if(playing){if(selected>=poses.length-1)setSelected(0);timer=setInterval(()=>{if(selected>=poses.length-1){playing=false;clearInterval(timer);$('#play').textContent='▶ 播放';return}setSelected(selected+1)},Math.max(35,220-Number($('#speed').value)*18))}else clearInterval(timer)};
canvas.addEventListener('wheel',e=>{e.preventDefault();const b=canvas.getBoundingClientRect();zoom(e.deltaY<0?1.12:1/1.12,e.clientX-b.left,e.clientY-b.top)},{passive:false});canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,ox:view.ox,oy:view.oy,moved:false};canvas.setPointerCapture(e.pointerId)});canvas.addEventListener('pointermove',e=>{if(!drag)return;if(Math.hypot(e.clientX-drag.x,e.clientY-drag.y)>3)drag.moved=true;if(drag.moved){view.ox=drag.ox+e.clientX-drag.x;view.oy=drag.oy+e.clientY-drag.y;draw()}});canvas.addEventListener('pointerup',e=>{if(drag&&!drag.moved){const b=canvas.getBoundingClientRect(),x=e.clientX-b.left,y=e.clientY-b.top;let best=-1,dist=14;poses.forEach((p,i)=>{const q=screen(p),d=Math.hypot(q.x-x,q.y-y);if(d<dist){best=i;dist=d}});if(best>=0)setSelected(best)}drag=null});window.addEventListener('resize',resize);
$('#file').onchange=async e=>{const f=e.target.files[0];if(!f)return;try{load(JSON.parse(await f.text()),f.name)}catch(err){alert('无法读取轨迹：'+err.message)}e.target.value=''};
fetch('/api/path').then(r=>{if(!r.ok)throw Error('HTTP '+r.status);return r.json()}).then(d=>load({poses:d.poses,frame_id:d.frame_id},d.name)).catch(e=>{$('#name').textContent='加载失败';alert('轨迹加载失败：'+e.message)});
resize();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    path_file: Path

    def do_GET(self) -> None:
        route = urlparse(self.path).path
        if route == "/":
            body = PAGE.encode("utf-8")
            content_type = "text/html; charset=utf-8"
            status = 200
        elif route == "/api/path":
            try:
                data = json.loads(self.path_file.read_text(encoding="utf-8"))
                if not isinstance(data, dict) or not isinstance(data.get("poses"), list):
                    raise ValueError("JSON 必须包含 poses 数组")
                data["name"] = self.path_file.name
                body = json.dumps(data, ensure_ascii=False).encode("utf-8")
                content_type = "application/json; charset=utf-8"
                status = 200
            except (OSError, json.JSONDecodeError, ValueError) as exc:
                body = json.dumps({"error": str(exc)}, ensure_ascii=False).encode("utf-8")
                content_type = "application/json; charset=utf-8"
                status = 400
        else:
            body = b"Not found"
            content_type = "text/plain; charset=utf-8"
            status = 404
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: object) -> None:
        print(f"[{self.log_date_time_string()}] {fmt % args}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, default=DEFAULT_PATH, help="位姿 JSON 文件")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址，默认仅本机访问")
    parser.add_argument("--port", type=int, default=8765, help="监听端口，默认 8765")
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    args = parser.parse_args()
    path_file = args.path.expanduser().resolve()
    if not path_file.is_file():
        parser.error(f"找不到轨迹文件: {path_file}")
    try:
        data = json.loads(path_file.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("poses"), list) or not data["poses"]:
            parser.error("JSON 必须包含非空 poses 数组")
    except (OSError, json.JSONDecodeError) as exc:
        parser.error(f"无法读取轨迹 JSON: {exc}")

    handler = type("PathHandler", (Handler,), {"path_file": path_file})
    try:
        server = ThreadingHTTPServer((args.host, args.port), handler)
    except OSError as exc:
        parser.error(f"无法监听 {args.host}:{args.port}: {exc}")
    url = f"http://{args.host}:{server.server_port}/"
    print(f"路径文件: {path_file}\n浏览器地址: {url}\n按 Ctrl+C 停止服务")
    if not args.no_browser and args.host in ("127.0.0.1", "localhost", "::1"):
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n服务已停止")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
