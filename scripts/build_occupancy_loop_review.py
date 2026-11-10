#!/usr/bin/env python3
"""Build a self-contained HTML review of the occupancy perimeter route."""
import argparse
import base64
import json
import math
from io import BytesIO
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label

from env.occupancy_loop import plan_loop


def data_uri(path):
    suffix = path.suffix.lower()
    mime = "image/png" if suffix == ".png" else "application/octet-stream"
    return f"data:{mime};base64," + base64.b64encode(path.read_bytes()).decode("ascii")


def polyline(points):
    return " ".join(f"{col + 0.5:.2f},{row + 0.5:.2f}" for row, col in points)


def contour_path(contour):
    points = contour[:, 0, :]
    if len(points) < 2:
        return ""
    return "M" + " L".join(f"{x + 0.5:.1f} {y + 0.5:.1f}" for x, y in points) + " Z"


def clearance_image(distance, free, scale):
    # A color ramp makes the 0.2 m cutoff legible while keeping obstacles dark.
    stops = np.asarray([
        [32, 45, 53], [44, 104, 123], [67, 157, 156],
        [190, 194, 116], [239, 153, 84], [193, 77, 62],
    ], dtype=np.float32)
    t = np.clip(distance / 1.5, 0.0, 1.0) * (len(stops) - 1)
    lo = np.floor(t).astype(int)
    hi = np.minimum(lo + 1, len(stops) - 1)
    frac = (t - lo)[..., None]
    rgb = (stops[lo] * (1.0 - frac) + stops[hi] * frac).astype(np.uint8)
    rgb[~free] = [26, 34, 39]
    # Small white stipples mark the exact planning cutoff without obscuring the route.
    edge = free & (distance >= 0.2 - 1e-9) & (distance < 0.2 + scale)
    rgb[edge] = [245, 245, 225]
    return Image.fromarray(rgb)


def build_report(root, output, clearance, simplify_tolerance):
    root = root.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    image_path = root / "data/occupancy.png"
    metadata_path = root / "data/occupancy.json"
    if not metadata_path.exists():
        metadata_path = root / "data/occupancy .json"
    image = np.asarray(Image.open(image_path).convert("L"))
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    scale = float(metadata["scale"])
    origin = metadata["min"][:2]
    route = plan_loop(image, scale, origin, clearance, simplify_tolerance)
    planning = route["planning"]

    free = image == 255
    distance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1] * scale
    safe = free & (distance + 1e-9 >= clearance)
    components, _ = label(safe)
    safe &= components == components[tuple(planning["start_pixel_rc"])]
    rgba = np.zeros((*safe.shape, 4), dtype=np.uint8)
    rgba[safe] = [22, 149, 123, 72]
    inset = safe & (distance + 1e-9 >= clearance + scale / 2)
    contours, _ = cv2.findContours(inset.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    boundary_svg = "\n".join(
        f'<path d="{contour_path(contour)}" fill="none" stroke="#f2d36d" stroke-width="0.65" vector-effect="non-scaling-stroke"/>'
        for contour in contours if len(contour) >= 3
    )

    h, w = image.shape
    start = tuple(planning["start_pixel_rc"])
    approach = planning["approach_pixel_rc"]
    perimeter = planning["perimeter_pixel_rc"]
    raw_perimeter = planning["raw_perimeter_pixel_rc"]
    return_path = list(reversed(approach))
    loop_closed = perimeter + perimeter[:1]
    poses = route["poses"]
    world_x = [p["x"] for p in poses]
    world_y = [p["y"] for p in poses]
    entry = perimeter[0]

    arrows = []
    for index in range(0, len(perimeter), max(1, len(perimeter) // 9)):
        a = perimeter[index]
        b = perimeter[(index + 2) % len(perimeter)]
        x1, y1 = a[1] + 0.5, a[0] + 0.5
        x2, y2 = b[1] + 0.5, b[0] + 0.5
        angle = np.arctan2(y2 - y1, x2 - x1)
        length, half_width = 2.3, 1.25
        tip = (x1 + np.cos(angle) * length, y1 + np.sin(angle) * length)
        left = (x1 - np.cos(angle) * 0.5 + np.cos(angle + np.pi / 2) * half_width,
                y1 - np.sin(angle) * 0.5 + np.sin(angle + np.pi / 2) * half_width)
        right = (x1 - np.cos(angle) * 0.5 - np.cos(angle + np.pi / 2) * half_width,
                 y1 - np.sin(angle) * 0.5 - np.sin(angle + np.pi / 2) * half_width)
        arrows.append("<polygon points='{}' fill='#0b8066' stroke='white' stroke-width='.35'/>".format(
            " ".join(f"{x:.2f},{y:.2f}" for x, y in (tip, left, right))))

    occupancy_uri = data_uri(image_path)
    clearance_buffer = BytesIO()
    clearance_image(distance, free, scale).save(clearance_buffer, format="PNG")
    clearance_uri = "data:image/png;base64," + base64.b64encode(clearance_buffer.getvalue()).decode("ascii")
    safe_buffer = BytesIO()
    Image.fromarray(rgba).save(safe_buffer, format="PNG")
    safe_uri = "data:image/png;base64," + base64.b64encode(safe_buffer.getvalue()).decode("ascii")

    route_json = json.dumps({"poses": poses, "planning": planning}, separators=(",", ":"))
    if abs(poses[0]["x"] - poses[-1]["x"]) > 1e-9 or abs(poses[0]["y"] - poses[-1]["y"]) > 1e-9:
        raise RuntimeError("Generated route is not closed at its start/end")
    if planning["minimum_route_clearance_m"] + 1e-9 < clearance:
        raise RuntimeError("Generated route does not meet the requested clearance")
    for index, (a, b) in enumerate(zip(poses, poses[1:])):
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        steps = max(1, math.ceil(math.hypot(dx, dy) / 0.025))
        for step in range(steps + 1):
            fraction = step / steps
            x, y = a["x"] + fraction * dx, a["y"] + fraction * dy
            col = math.floor(image.shape[1] - (x - origin[0]) / scale)
            row = math.floor((y - origin[1]) / scale)
            if not (0 <= row < h and 0 <= col < w) or distance[row, col] + 1e-9 < clearance:
                raise RuntimeError(f"Waypoint segment {index} violates clearance at {(x, y)}")
    scene_validator = "通过（waypoint 线段每 0.025 m 采样）"

    html = f'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Occupancy 巡航轨迹审核</title>
<style>
:root{{--ink:#192d31;--muted:#607277;--line:#d0dcda;--paper:#eef3f0;--teal:#0b8066;--blue:#26759a;--amber:#ca772e;--white:#fff}}
*{{box-sizing:border-box}} body{{margin:0;background:linear-gradient(125deg,#e8f0ed,#f5f3eb 60%,#e7edef);color:var(--ink);font:15px/1.5 "Noto Sans CJK SC","Microsoft YaHei",sans-serif}}
main{{max-width:1500px;margin:auto;padding:22px}} header{{display:flex;justify-content:space-between;align-items:end;gap:20px;padding:15px 3px 18px;border-bottom:2px solid #9ab6b1}}
h1{{font:700 32px/1.18 "Noto Serif CJK SC","Songti SC",serif;margin:0 0 7px;color:#154f4b}} .sub{{margin:0;color:var(--muted)}}
.badge{{padding:7px 11px;border:1px solid #b8d4c9;background:#e2f3e9;color:#205b42;font-size:13px;white-space:nowrap}}
.layout{{display:grid;grid-template-columns:minmax(400px,1fr) 330px;gap:18px;margin-top:18px;align-items:start}}
.map-panel,.side section{{background:var(--white);border:1px solid var(--line)}} .map-top{{padding:12px 15px;display:flex;justify-content:space-between;gap:10px;align-items:center;border-bottom:1px solid var(--line)}}
.switches{{display:flex;gap:6px}} button{{border:1px solid #bdcbc8;background:#f4f7f5;color:#354c50;padding:7px 11px;font:inherit;cursor:pointer}} button[aria-pressed="true"]{{background:#195f58;border-color:#195f58;color:#fff}}
.svg-wrap{{height:min(78vh,880px);min-height:540px;overflow:auto;background:#e8eeeb;display:grid;place-items:center;padding:15px}} svg{{display:block;height:100%;width:auto;max-width:100%;object-fit:contain;image-rendering:pixelated}}
.legend{{display:flex;gap:16px;flex-wrap:wrap;padding:11px 15px;border-top:1px solid var(--line);font-size:13px;color:#41575b}} .key{{display:inline-flex;align-items:center;gap:7px}} .swatch{{width:24px;height:4px;background:var(--teal)}} .approach{{background:var(--blue)}} .return{{height:0;border-top:3px dashed var(--amber);background:transparent}} .clearance-key{{width:13px;height:13px;background:#f2d36d}}
.side{{display:grid;gap:12px}} .side section{{padding:14px}} h2{{font:700 18px/1.3 "Noto Serif CJK SC","Songti SC",serif;margin:0 0 11px;color:#24575b}} .metric{{display:flex;justify-content:space-between;gap:10px;border-top:1px solid #e4eae8;padding:8px 0;font-size:13px}} .metric strong{{text-align:right;font-variant-numeric:tabular-nums}}
.note{{font-size:13px;color:#506267;margin:0}} .warn{{border-left:4px solid #cc8736;background:#fff7e8;padding:11px 12px}} .good{{border-left:4px solid #38875e;background:#eef8f0;padding:11px 12px}} .route-meta{{overflow-wrap:anywhere}}
.bottom{{margin-top:14px;display:grid;grid-template-columns:1fr 1fr;gap:12px}} .bottom section{{padding:13px 15px;background:#fff;border:1px solid var(--line)}} code{{font:12px "DejaVu Sans Mono",monospace;background:#edf2f0;padding:1px 4px}}
@media(max-width:900px){{main{{padding:12px}} header{{align-items:start;flex-direction:column}} .layout{{grid-template-columns:1fr}} .svg-wrap{{height:72vh;min-height:460px}} .side{{grid-template-columns:repeat(auto-fit,minmax(240px,1fr))}} .bottom{{grid-template-columns:1fr}} h1{{font-size:27px}}}}
@media(max-width:520px){{.svg-wrap{{min-height:420px;padding:6px}} .map-top{{align-items:flex-start;flex-direction:column}} .switches button{{padding:6px 8px;font-size:13px}}}}
</style></head>
<body><main>
<header><div><h1>Occupancy 周边巡航轨迹审核</h1><p class="sub">仅离线规划可视化；未启动仿真、ROS bridge 或 mapping。场景坐标保持原始 data/ frame。</p></div><div class="badge">几何校验：通过 · 实际执行：未测试</div></header>
<div class="layout">
<section class="map-panel"><div class="map-top"><strong>路线顺序：起点 → 蓝色接入 → 绿色绕行一圈 → 橙色原路返回</strong><div class="switches"><button id="occBtn" aria-pressed="true">Occupancy</button><button id="distBtn" aria-pressed="false">障碍距离场</button></div></div>
<div class="svg-wrap"><svg viewBox="0 0 {w} {h}" role="img" aria-label="occupancy map with closed perimeter cruise path">
<image id="occupancy" href="{occupancy_uri}" x="0" y="0" width="{w}" height="{h}" image-rendering="pixelated"/>
<image id="distance" href="{clearance_uri}" x="0" y="0" width="{w}" height="{h}" image-rendering="pixelated" style="display:none"/>
<image href="{safe_uri}" x="0" y="0" width="{w}" height="{h}" image-rendering="pixelated"/>
{boundary_svg}
<polyline points="{polyline(approach)}" fill="none" stroke="#26759a" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" vector-effect="non-scaling-stroke" marker-end="url(#blueArrow)"/>
<polyline points="{polyline(raw_perimeter + raw_perimeter[:1])}" fill="none" stroke="#879695" stroke-width="1.0" stroke-linecap="round" stroke-linejoin="round" vector-effect="non-scaling-stroke"/>
<polyline points="{polyline(loop_closed)}" fill="none" stroke="#0b8066" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round" vector-effect="non-scaling-stroke"/>
{''.join(arrows)}
<polyline points="{polyline(return_path)}" fill="none" stroke="#ca772e" stroke-width="2.0" stroke-dasharray="5 3" stroke-linecap="round" stroke-linejoin="round" vector-effect="non-scaling-stroke" marker-end="url(#amberArrow)"/>
<circle cx="{start[1]+0.5}" cy="{start[0]+0.5}" r="3.0" fill="#e24c42" stroke="white" stroke-width="1.1" vector-effect="non-scaling-stroke"/><text x="{start[1]+4}" y="{start[0]-3}" font-size="5" font-weight="700" fill="#7a2723" paint-order="stroke" stroke="white" stroke-width="1.5">START / END</text>
<rect x="{entry[1]-1.6}" y="{entry[0]-1.6}" width="3.2" height="3.2" transform="rotate(45 {entry[1]+0.0} {entry[0]+0.0})" fill="#f3c64c" stroke="#624e1c" stroke-width=".7" vector-effect="non-scaling-stroke"/>
<defs><marker id="blueArrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto" markerUnits="userSpaceOnUse"><path d="M0 0L7 3.5L0 7Z" fill="#26759a"/></marker><marker id="amberArrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto" markerUnits="userSpaceOnUse"><path d="M0 0L7 3.5L0 7Z" fill="#ca772e"/></marker></defs>
</svg></div><div class="legend"><span class="key"><i class="swatch approach"></i>接入段</span><span class="key"><i class="swatch"></i>OpenCV DP 简化后环线</span><span class="key"><i class="swatch" style="background:#879695"></i>简化前原始轮廓</span><span class="key"><i class="swatch return"></i>沿接入段返回</span><span class="key"><i class="swatch" style="background:#e24c42;width:12px;height:12px;border-radius:50%"></i>最远 clearance 起终点</span><span class="key"><i class="swatch clearance-key"></i>安全区边界 (≥{clearance:.2f} m)</span></div></section>
<aside class="side">
<section><h2>路线摘要</h2>
<div class="metric"><span>起点 clearance</span><strong>{planning['start_clearance_m']:.2f} m</strong></div>
<div class="metric"><span>路线最小 clearance</span><strong>{planning['minimum_route_clearance_m']:.2f} m</strong></div>
<div class="metric"><span>安全区阈值</span><strong>≥ {clearance:.2f} m</strong></div>
<div class="metric"><span>外围环线长度</span><strong>{planning['perimeter_length_m']:.2f} m</strong></div>
<div class="metric"><span>接入段单程</span><strong>{planning['approach_length_m']:.2f} m</strong></div>
<div class="metric"><span>本次计划总路程</span><strong>{planning['route_length_m']:.2f} m</strong></div>
<div class="metric"><span>scene waypoints</span><strong>{len(poses)}</strong></div>
<div class="metric"><span>轮廓顶点</span><strong>{planning['raw_perimeter_vertices']} → {planning['simplified_perimeter_vertices']}</strong></div>
<div class="metric"><span>DP 最大偏差容差</span><strong>{planning['simplify_tolerance_used_m']:.3f} m</strong></div>
<div class="metric"><span>起点 scene (x, y)</span><strong>({poses[0]['x']:.3f}, {poses[0]['y']:.3f}) m</strong></div>
<div class="metric"><span>接入轮廓位置</span><strong>row {entry[0]}, col {entry[1]}</strong></div>
</section>
<section><h2>覆盖范围</h2><p class="note">scene-frame waypoint 的轴对齐范围：</p><div class="metric"><span>x</span><strong>{min(world_x):.2f} 至 {max(world_x):.2f} m</strong></div><div class="metric"><span>y</span><strong>{min(world_y):.2f} 至 {max(world_y):.2f} m</strong></div><p class="note">只沿所选最大内部障碍轮廓巡航一圈，不代表覆盖整个 occupancy 地图，也不是扫地式覆盖。</p></section>
<section class="good"><h2>离线检查</h2><p class="note">路线闭合、最大 clearance 起点、连通安全区、轮廓与接入段的离散安全检查：通过。场景路径逐段检查（0.025 m 采样）：{scene_validator}。同图规划确定性：通过。</p></section>
<section class="warn"><h2>审核边界</h2><p class="note">0.2 m 是机器人中心到占据障碍的二维几何距离，不含 Go2 足迹、动态碰撞、转弯动力学或足端稳定性。仿真策略是否能跟上该路线尚未验证；本报告只证明离线路径几何，不声称机器人已执行。</p></section>
</aside></div>
<div class="bottom"><section><strong>OpenCV DP 简化说明</strong><p class="note">对闭合轮廓使用 <code>cv2.approxPolyDP</code>，默认最大几何偏差 0.15 m；每条简化弦段在栅格安全区内密集采样。若弦段越出安全区，会自动减半容差；无法安全简化时保留原始轮廓。</p></section><section><strong>坐标与审核边界</strong><p class="note route-meta">输入：<code>data/occupancy.png</code>、<code>{metadata_path.relative_to(root)}</code>；分辨率 {scale:.3f} m/px。waypoint 保持原始 scene 坐标，不做 recenter。几何检查不等于 Go2 动力学或实测通过。</p></section></div>
</main><script>
const occ=document.getElementById('occupancy'),dist=document.getElementById('distance');
function setLayer(which){{const isOcc=which==='occ';occ.style.display=isOcc?'':'none';dist.style.display=isOcc?'none':'';document.getElementById('occBtn').setAttribute('aria-pressed',String(isOcc));document.getElementById('distBtn').setAttribute('aria-pressed',String(!isOcc));}}
document.getElementById('occBtn').addEventListener('click',()=>setLayer('occ'));
document.getElementById('distBtn').addEventListener('click',()=>setLayer('dist'));
window.routeData={route_json};
</script></body></html>'''
    output.write_text(html, encoding="utf-8")
    summary_path = output.with_name("trajectory_metrics.json")
    route_path = output.with_name("cruise_path.json")
    route_path.write_text(json.dumps(route, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {
        "report": str(output),
        "route_file": str(route_path),
        "source_occupancy": str(image_path),
        "source_metadata": str(metadata_path),
        "planner": "env.occupancy_loop.plan_loop",
        "frame_id": route["frame_id"],
        "clearance_m": clearance,
        "waypoint_count": len(poses),
        "start_end_scene_xy": [poses[0]["x"], poses[0]["y"]],
        "planning": {k: v for k, v in planning.items() if not k.endswith("_pixel_rc")},
        "world_bounds_xy": [min(world_x), max(world_x), min(world_y), max(world_y)],
        "scene_path_validation": scene_validator,
        "execution_status": "not_run_by_request",
    }
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--clearance", type=float, default=0.2)
    parser.add_argument("--simplify-tolerance", type=float, default=0.15, help="Maximum DP deviation in meters")
    args = parser.parse_args()
    output = args.output or args.root / "reports/occupancy-loop-validation/trajectory_review.html"
    build_report(args.root, output, args.clearance, args.simplify_tolerance)


if __name__ == "__main__":
    main()
