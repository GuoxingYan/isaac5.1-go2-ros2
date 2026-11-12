"""Render a live cruise route and simulator pose over an occupancy image."""
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def _pixel(x, y, shape, scale, origin):
    height, width = shape[:2]
    col = width - (float(x) - origin[0]) / scale - 0.5
    row = (float(y) - origin[1]) / scale - 0.5
    return int(round(col)), int(round(row))


def render_occupancy_overlay(image_path, metadata, path_data, output_path,
                             robot_pose=None, target_index=None, current_index=None,
                             title=None, scale_factor=2):
    """Write an annotated PNG; coordinates follow InteriorGSScene's map convention."""
    source = Image.open(image_path).convert("RGB")
    base = source.resize((source.width * scale_factor, source.height * scale_factor), Image.Resampling.NEAREST)
    margin_top = 32 if title else 0
    if margin_top:
        canvas = Image.new("RGB", (base.width, base.height + margin_top), (28, 34, 38))
        canvas.paste(base, (0, margin_top))
        base = canvas
    draw = ImageDraw.Draw(base)
    scale = float(metadata["scale"])
    origin = np.asarray(metadata["min"][:2], dtype=float)
    poses = path_data.get("poses", [])
    points = [tuple(v * scale_factor for v in _pixel(p["x"], p["y"], source.size[::-1], scale, origin))
              for p in poses]
    points = [(x, y + margin_top) for x, y in points]

    if len(points) > 1:
        route = points + ([points[0]] if path_data.get("loop", False) else [])
        draw.line(route, fill=(0, 190, 255), width=3)
        for index, point in enumerate(points):
            radius = 6
            color = (255, 190, 0) if index == target_index else (0, 210, 120)
            draw.ellipse((point[0]-radius, point[1]-radius, point[0]+radius, point[1]+radius),
                         fill=color, outline=(20, 20, 20), width=2)
            # Alternate label sides so closely spaced perimeter points stay readable.
            dx = 8 if index % 2 == 0 else -20
            dy = -11 if index % 4 < 2 else 4
            draw.text((point[0]+dx, point[1]+dy), str(index), fill=(255, 255, 255),
                      stroke_width=1, stroke_fill=(0, 0, 0))
    if current_index is not None and target_index is not None and points:
        start = points[current_index]
        end = points[target_index]
        draw.line((start, end), fill=(255, 80, 30), width=5)
    if robot_pose is not None:
        x, y, yaw = map(float, robot_pose)
        center = _pixel(x, y, source.size[::-1], scale, origin)
        tip = _pixel(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw),
                     source.size[::-1], scale, origin)
        center = (center[0] * scale_factor, center[1] * scale_factor + margin_top)
        tip = (tip[0] * scale_factor, tip[1] * scale_factor + margin_top)
        draw.ellipse((center[0]-8, center[1]-8, center[0]+8, center[1]+8),
                     fill=(255, 40, 40), outline=(255, 255, 255), width=2)
        draw.line((center, tip), fill=(255, 40, 40), width=5)
    if title:
        draw.text((10, 9), title, fill=(245, 248, 250), stroke_width=0)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    base.save(temporary, format="PNG")
    temporary.replace(output_path)
    return output_path
