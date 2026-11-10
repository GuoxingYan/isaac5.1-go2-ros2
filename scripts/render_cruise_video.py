#!/usr/bin/env python3
"""Render Gaussian RGB frames from a recorded simulator truth trajectory."""
import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation, Slerp

from env.gaussian_rgb import GaussianRGBRenderer
from go2.sensor_profile import PROFILE


def load_trace(path):
    rows = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.strip():
            row = json.loads(line)
            if row.get("sim_finished"):
                continue
            rows.append(row)
    if len(rows) < 2:
        raise ValueError("Need at least two recorded cruise poses")
    return rows


def camera_pose(row):
    body_position = np.asarray(row["position"], dtype=np.float64)
    body_rotation = Rotation.from_quat(np.asarray(row["quaternion_wxyz"], dtype=np.float64)[[1, 2, 3, 0]])
    camera_offset = np.asarray(PROFILE["camera"]["position"], dtype=np.float64)
    # Isaac Camera.set_world_pose(camera_axes='world') followed by
    # get_world_pose(camera_axes='ros'): W_U_TRANSFORM @ U_R_TRANSFORM.
    world_to_optical = Rotation.from_matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    camera_rotation = body_rotation * Rotation.from_euler("xyz", PROFILE["camera"]["rpy_deg"], degrees=True) * world_to_optical
    camera_position = body_position + body_rotation.apply(camera_offset)
    quaternion_xyzw = camera_rotation.as_quat()
    return camera_position, quaternion_xyzw[[3, 0, 1, 2]]


def choose_frames(rows, fps):
    times = np.asarray([float(row["simulation_time"]) for row in rows])
    if np.any(np.diff(times) <= 0):
        raise ValueError("Trajectory simulation timestamps must be strictly increasing")
    targets = np.arange(times[0], times[-1] + 1e-9, 1.0 / fps)
    positions = np.asarray([row["position"] for row in rows])
    quaternions = np.asarray([row["quaternion_wxyz"] for row in rows])[:, [1, 2, 3, 0]]
    interpolated_quaternions = Slerp(times, Rotation.from_quat(quaternions))(targets).as_quat()[:, [3, 0, 1, 2]]
    interpolated_positions = np.column_stack([np.interp(targets, times, positions[:, i]) for i in range(3)])
    return [{"simulation_time": t, "position": p, "quaternion_wxyz": q}
            for t, p, q in zip(targets, interpolated_positions, interpolated_quaternions)]


def render_video(trace_path, data_dir, output, fps, resolution, crf):
    rows = load_trace(trace_path)
    selected = choose_frames(rows, fps)
    width, height = resolution
    cfg = PROFILE["camera"]
    focal = width / (2.0 * math.tan(math.radians(float(cfg["rgb_hfov_deg"])) / 2.0))
    intrinsics = np.asarray([[focal, 0., (width - 1.) / 2.],
                             [0., focal, (height - 1.) / 2.],
                             [0., 0., 1.]], dtype=np.float32)
    renderer = GaussianRGBRenderer(Path(data_dir) / "839873.usdz")
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="go2-gaussian-frames-") as temp_dir:
        temp_dir = Path(temp_dir)
        for index, row in enumerate(selected):
            position, quaternion = camera_pose(row)
            image = renderer.render(position, quaternion, intrinsics, (width, height))
            from PIL import Image
            Image.fromarray(image).save(temp_dir / f"frame_{index:06d}.png")
        command = [
            "ffmpeg", "-y", "-framerate", str(fps), "-i", str(temp_dir / "frame_%06d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", str(crf), "-movflags", "+faststart", str(output),
        ]
        subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    metadata = {
        "video": str(output), "fps": fps, "frames": len(selected),
        "duration_s": len(selected) / fps, "resolution": [width, height],
        "trajectory_sim_time_s": [float(rows[0]["simulation_time"]), float(rows[-1]["simulation_time"])],
        "source": "offline replay of recorded simulator truth pose + data/839873.usdz Gaussian renderer",
        "frame_interpolation": "linear position and quaternion SLERP between recorded truth samples",
        "camera_mount": {"position": cfg["position"], "rpy_deg": cfg["rpy_deg"], "hfov_deg": cfg["rgb_hfov_deg"]},
        "no_recenter": True,
    }
    output.with_suffix(".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--data-dir", type=Path, default=Path(__file__).resolve().parents[1] / "data")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--crf", type=int, default=20)
    args = parser.parse_args()
    if not torch.cuda.is_available():
        raise RuntimeError("Gaussian video rendering requires CUDA")
    render_video(args.trace, args.data_dir, args.output, args.fps, (args.width, args.height), args.crf)


if __name__ == "__main__":
    main()
