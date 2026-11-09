"""Render the supplied InteriorGS USDZ with its embedded NuRec Gaussians."""
import gzip
import hashlib
import time
import zipfile
from pathlib import Path

import msgpack
import numpy as np
import torch
from scipy.spatial.transform import Rotation


class GaussianRGBRenderer:
    """Use the source Gaussian asset for RGB while geometry stays in the mesh."""

    def __init__(self, path, translation=(0.0, 0.0, 0.0)):
        from gsplat import rasterization

        self.path = Path(path).resolve()
        self.rasterization = rasterization
        with zipfile.ZipFile(self.path) as archive:
            member = next(name for name in archive.namelist() if name.endswith(".nurec"))
            packed = gzip.decompress(archive.read(member))
        state = msgpack.unpackb(packed, raw=False, strict_map_key=False)["nre_data"]["state_dict"]

        def tensor(name):
            key = ".gaussians_nodes.gaussians." + name
            values = np.frombuffer(state[key], np.float16).reshape(state[key + ".shape"]).astype(np.float32)
            return torch.from_numpy(values).cuda()

        self.means = tensor("positions")
        self.means += torch.as_tensor(translation, dtype=self.means.dtype, device=self.means.device)
        self.quats = torch.nn.functional.normalize(tensor("rotations"), dim=-1)
        self.scales = torch.exp(tensor("scales"))
        self.opacities = torch.sigmoid(tensor("densities")).flatten()
        dc = tensor("features_albedo")
        rest = tensor("features_specular").reshape(len(dc), 15, 3)
        self.colors = torch.cat([dc[:, None, :], rest], dim=1).contiguous()
        self.frames = 0
        self.timings = []
        self.last_image = None
        self.last_pose = None
        self.info = {
            "source": str(self.path),
            "member": member,
            "gaussians": int(len(self.means)),
            "sh_degree": 3,
            "translation_xyz_m": [float(x) for x in translation],
            "renderer": "gsplat CUDA anisotropic 3D Gaussian rasterization",
            "coordinate_frame": "data/ source frame; no recentering or translation",
            "sha256": hashlib.sha256(self.path.read_bytes()).hexdigest(),
            "limits": "RGB only; collision mesh supplies metric depth and LiDAR.",
        }

    @torch.inference_mode()
    def render(self, position, quaternion_wxyz, intrinsics, resolution):
        started = time.perf_counter()
        position = np.asarray(position, dtype=np.float64)
        quaternion_wxyz = np.asarray(quaternion_wxyz, dtype=np.float64)
        rotation = Rotation.from_quat(quaternion_wxyz[[1, 2, 3, 0]]).as_matrix()
        view = np.eye(4, dtype=np.float32)
        view[:3, :3] = rotation.T
        view[:3, 3] = -rotation.T @ position
        camera_matrix = torch.as_tensor(np.asarray(intrinsics), dtype=torch.float32, device="cuda")[None]
        image, alpha, _ = self.rasterization(
            self.means,
            self.quats,
            self.scales,
            self.opacities,
            self.colors,
            torch.as_tensor(view, device="cuda")[None],
            camera_matrix,
            int(resolution[0]),
            int(resolution[1]),
            sh_degree=3,
            near_plane=0.05,
            far_plane=100.0,
            packed=True,
        )
        rgb = (image[0].clamp(0, 1) * 255).to(torch.uint8).cpu().numpy()
        self.last_image = rgb.copy()
        self.last_alpha = alpha[0, :, :, 0].cpu().numpy()
        self.last_pose = {
            "position": position.tolist(),
            "quaternion_wxyz": quaternion_wxyz.tolist(),
            "intrinsics": np.asarray(intrinsics).tolist(),
        }
        self.frames += 1
        self.timings.append(time.perf_counter() - started)
        return rgb
