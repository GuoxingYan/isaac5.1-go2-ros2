#!/usr/bin/env python3
"""将 USD 碰撞网格采样为 PCD 点云。"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import ctypes
import subprocess
from pathlib import Path

import numpy as np


def _import_usd_modules():
    """Load Isaac Sim's bundled USD Python modules when conda omits pxr."""
    try:
        from pxr import Usd, UsdGeom
        return Usd, UsdGeom
    except ImportError:
        pass

    roots = [Path(sys.prefix), Path(sys.executable).resolve().parent.parent]
    candidates = []
    for root in dict.fromkeys(roots):
        candidates.extend(glob.glob(str(root / "lib/python*/site-packages/isaacsim/extscache/omni.usd.libs-*/pxr")))
        candidates.extend(glob.glob(str(root / "lib/python*/site-packages/pxr")))

    for pxr_dir in candidates:
        pxr_path = Path(pxr_dir)
        site_packages = pxr_path.parent
        if pxr_path.name != "pxr":
            continue
        sys.path.insert(0, str(site_packages))
        library_dirs = [str(pxr_path.parent / "bin"), str(Path(sys.prefix) / "lib")]
        library_dirs.extend(glob.glob(str(site_packages / "isaacsim" / "kit" / "lib")))
        library_dirs.extend(glob.glob(str(site_packages / "isaacsim" / "kit")))
        library_dirs.extend(glob.glob(str(site_packages / "isaacsim" / "extscache" / "omni.usd.libs-*" / "bin")))
        current = os.environ.get("LD_LIBRARY_PATH", "")
        updated = os.pathsep.join(library_dirs + ([current] if current else []))
        os.environ["LD_LIBRARY_PATH"] = updated
        if sys.platform.startswith("linux") and not os.environ.get("USD_TO_PCD_REEXEC"):
            env = os.environ.copy()
            env["USD_TO_PCD_REEXEC"] = "1"
            result = subprocess.run([sys.executable, *sys.argv], env=env)
            raise SystemExit(result.returncode)
        if hasattr(os, "add_dll_directory"):
            for directory in library_dirs:
                os.add_dll_directory(directory)
        try:
            from pxr import Usd, UsdGeom
            return Usd, UsdGeom
        except ImportError:
            continue

    raise RuntimeError(
        "未找到 pxr。请确认当前 Python 环境安装了 Isaac Sim，或使用 Isaac Sim 自带的 Python 运行。"
    )


def _load_usd_meshes(input_path: Path) -> list[tuple[np.ndarray, np.ndarray]]:
    """读取 USD 中的 Mesh，并把顶点转换到世界坐标。"""
    Usd, UsdGeom = _import_usd_modules()

    stage = Usd.Stage.Open(str(input_path))
    if not stage:
        raise RuntimeError(f"无法打开 USD 文件: {input_path}")

    xform_cache = UsdGeom.XformCache()
    meshes: list[tuple[np.ndarray, np.ndarray]] = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue

        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get() or []
        counts = mesh.GetFaceVertexCountsAttr().Get() or []
        indices = mesh.GetFaceVertexIndicesAttr().Get() or []
        if not points or not counts or not indices:
            continue

        transform = xform_cache.GetLocalToWorldTransform(prim)
        vertices = np.asarray(
            [
                [*transform.Transform(point)[:3]]
                for point in points
            ],
            dtype=np.float64,
        )
        faces: list[tuple[int, int, int]] = []
        offset = 0
        for count in counts:
            polygon = indices[offset : offset + count]
            offset += count
            # USD Mesh 可以包含多边形，扇形三角化足以覆盖碰撞网格。
            for index in range(1, len(polygon) - 1):
                faces.append((polygon[0], polygon[index], polygon[index + 1]))
        if faces:
            meshes.append((vertices, np.asarray(faces, dtype=np.int64)))

    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    if meters_per_unit != 1.0:
        meshes = [(vertices * meters_per_unit, faces) for vertices, faces in meshes]
    return meshes


def _sample_meshes(
    meshes: list[tuple[np.ndarray, np.ndarray]], resolution: float
) -> np.ndarray:
    """按分辨率采样网格表面，并进行体素去重。"""
    samples: list[np.ndarray] = []
    for vertices, faces in meshes:
        for first, second, third in faces:
            a, b, c = vertices[first], vertices[second], vertices[third]
            longest_edge = max(
                np.linalg.norm(a - b),
                np.linalg.norm(b - c),
                np.linalg.norm(c - a),
            )
            subdivisions = max(1, math.ceil(longest_edge / resolution))
            for i in range(subdivisions + 1):
                for j in range(subdivisions + 1 - i):
                    weight_a = i / subdivisions
                    weight_b = j / subdivisions
                    samples.append(
                        (1.0 - weight_a - weight_b) * a
                        + weight_a * b
                        + weight_b * c
                    )

    if not samples:
        return np.empty((0, 3), dtype=np.float32)
    points = np.asarray(samples, dtype=np.float64)
    cells = np.floor(points / resolution).astype(np.int64)
    _, first_indices = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(first_indices)].astype(np.float32)


def _write_pcd(output_path: Path, points: np.ndarray) -> None:
    """以与 000001.pcd 一致的二进制格式写出点云。"""
    points_with_intensity = np.empty((len(points), 4), dtype="<f4")
    points_with_intensity[:, :3] = points
    points_with_intensity[:, 3] = 0.0
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    with output_path.open("wb") as file:
        file.write(header)
        file.write(points_with_intensity.tobytes(order="C"))


def convert(input_path: Path, output_path: Path, resolution: float) -> int:
    """执行 USD 到 PCD 的转换并返回点数。"""
    if resolution <= 0:
        raise ValueError("resolution 必须大于 0")
    meshes = _load_usd_meshes(input_path)
    points = _sample_meshes(meshes, resolution)
    if len(points) == 0:
        raise RuntimeError("USD 中没有可采样的 Mesh")
    _write_pcd(output_path, points)
    return len(points)


def main() -> None:
    """解析命令行参数并执行转换。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", nargs="?", type=Path, default=Path("/data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2/data/839873_collision.usd"),
        help="输入 USD 文件，默认: 839873_collision.usd",
    )
    parser.add_argument(
        "output", nargs="?", type=Path, default=Path("/data/projects/Sim2Real/codex_5.1/isaac5.1-go2-ros2/data/GlobalMap.pcd"),
        help="输出 PCD 文件，默认: 000001.pcd",
    )
    parser.add_argument(
        "--resolution", "-r", type=float, default=0.05,
        help="表面采样/体素去重分辨率（米），默认: 0.5",
    )
    args = parser.parse_args()
    point_count = convert(args.input, args.output, args.resolution)
    print(f"已生成 {args.output}，点数: {point_count}，分辨率: {args.resolution} m")


if __name__ == "__main__":
    main()
