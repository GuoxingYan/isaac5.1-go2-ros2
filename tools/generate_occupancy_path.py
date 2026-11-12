#!/usr/bin/env python3
"""Generate a path_3a02_loop-style pose JSON from an occupancy map."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import label

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from env.occupancy_loop import plan_loop


def _metadata_path(data_dir: Path) -> Path:
    candidates = (data_dir / "occupancy.json", data_dir / "occupancy .json")
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"Occupancy metadata not found; checked: {', '.join(str(p) for p in candidates)}"
    )


def generate_path(
    image_path: Path,
    metadata_path: Path,
    clearance: float = 0.2,
    overlay_path: Path | None = None,
) -> dict:
    """Plan an occupancy-safe route and convert its yaw values to quaternions."""
    image = np.asarray(Image.open(image_path).convert("L"))
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    scale = float(metadata["scale"])
    origin = metadata["min"][:2]
    if scale <= 0:
        raise ValueError("scale must be positive")

    route = plan_loop(image, scale, origin, clearance)
    route_points = route["poses"]
    min_x = min(float(point["x"]) for point in route_points)
    max_x = max(float(point["x"]) for point in route_points)
    min_y = min(float(point["y"]) for point in route_points)
    max_y = max(float(point["y"]) for point in route_points)
    rectangle = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
    rectangle.append(rectangle[0])

    def world_to_pixel(x: float, y: float) -> tuple[int, int]:
        col = int(math.floor(image.shape[1] - (x - origin[0]) / scale))
        row = int(math.floor((y - origin[1]) / scale))
        return row, col

    rectangle_pixels = [world_to_pixel(x, y) for x, y in rectangle]
    sampled_edges: list[tuple[int, int]] = []
    for start, end in zip(rectangle_pixels, rectangle_pixels[1:]):
        steps = max(abs(end[0] - start[0]), abs(end[1] - start[1]), 1)
        for step in range(steps + 1):
            t = step / steps
            sampled_edges.append((
                round(start[0] + t * (end[0] - start[0])),
                round(start[1] + t * (end[1] - start[1])),
            ))
    collisions = sorted({(row, col) for row, col in sampled_edges
                         if not (0 <= row < image.shape[0] and 0 <= col < image.shape[1])
                         or image[row, col] != 255})

    poses = []
    for i, (x, y) in enumerate(rectangle):
        next_x, next_y = rectangle[(i + 1) % len(rectangle)]
        yaw = math.atan2(next_y - y, next_x - x)
        half_yaw = yaw / 2.0
        poses.append(
            {
                "x": x,
                "y": y,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": math.sin(half_yaw),
                "qw": math.cos(half_yaw),
            }
        )
    path_data = {"poses": poses, "frame_id": route["frame_id"]}
    if overlay_path is not None:
        _write_route_overlay(
            image,
            route["planning"]["route_pixel_rc"],
            rectangle_pixels,
            collisions,
            overlay_path,
        )
    return path_data, len(collisions)


def _write_route_overlay(
    image: np.ndarray,
    route: list[list[int]],
    rectangle: list[tuple[int, int]],
    bbox_collisions: list[tuple[int, int]],
    output: Path,
) -> None:
    scale = 4
    base = Image.fromarray(image).convert("RGB").resize(
        (image.shape[1] * scale, image.shape[0] * scale), Image.Resampling.NEAREST
    )
    draw = ImageDraw.Draw(base)
    def xy(point: tuple[int, int] | list[int]) -> tuple[int, int]:
        row, col = point
        return (col * scale + scale // 2, row * scale + scale // 2)

    rect_xy = [xy(point) for point in rectangle]
    draw.line(rect_xy, fill=(255, 30, 30), width=4, joint="curve")
    for row, col in bbox_collisions:
        x, y = xy((row, col))
        draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=(255, 30, 30))
    points = [xy(point) for point in route]
    draw.line(points, fill=(0, 245, 255), width=3, joint="curve")
    for x, y in points:
        draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(0, 90, 255))
    output.parent.mkdir(parents=True, exist_ok=True)
    legend = Image.new("RGB", (base.width, 28), "white")
    ImageDraw.Draw(legend).text(
        (8, 7),
        f"BLUE: source safe route   RED: its axis-aligned bounding rectangle   RED dots: {len(bbox_collisions)} occupied/unknown samples",
        fill="black",
    )
    combined = Image.new("RGB", (base.width, base.height + legend.height), "white")
    combined.paste(base, (0, 0))
    combined.paste(legend, (0, base.height))
    combined.save(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data", help="occupancy 数据目录")
    parser.add_argument("--image", type=Path, default=None, help="occupancy PNG，默认使用 data-dir/occupancy.png")
    parser.add_argument("--metadata", type=Path, default=None, help="occupancy 元数据 JSON；自动兼容文件名中的空格")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "data" / "path_occupancy.json",
        help="输出路径 JSON，默认 data/path_occupancy.json",
    )
    parser.add_argument("--clearance", type=float, default=0.2, help="安全距离，单位米（默认 0.2）")
    parser.add_argument("--overlay", type=Path, default=ROOT / "reports" / "occupancy_path_overlay.png", help="写出路径叠加审核图")
    args = parser.parse_args()

    data_dir = args.data_dir.expanduser().resolve()
    image_path = (args.image or data_dir / "occupancy.png").expanduser().resolve()
    metadata_path = (args.metadata or _metadata_path(data_dir)).expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    if not image_path.is_file():
        parser.error(f"Occupancy image not found: {image_path}")
    if not metadata_path.is_file():
        parser.error(f"Occupancy metadata not found: {metadata_path}")

    try:
        path_data, collision_count = generate_path(
            image_path, metadata_path, args.clearance, args.overlay.expanduser().resolve()
        )
    except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError) as exc:
        if args.overlay.exists():
            print(f"Occupancy overlay: {args.overlay.expanduser().resolve()}", file=sys.stderr)
        parser.error(f"Could not generate occupancy path: {exc}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(path_data, indent=2) + "\n", encoding="utf-8")
    points = path_data["poses"]
    print(f"Generated {output_path}")
    print(f"Frame: {path_data['frame_id']}; poses: {len(points)}; z: 0.0")
    print(f"Occupancy overlay: {args.overlay.expanduser().resolve()}")
    print(f"Rectangle edge collision samples: {collision_count}")
    print(
        "XY bounds: "
        f"x=[{min(p['x'] for p in points):.3f}, {max(p['x'] for p in points):.3f}], "
        f"y=[{min(p['y'] for p in points):.3f}, {max(p['y'] for p in points):.3f}]"
    )


if __name__ == "__main__":
    main()
