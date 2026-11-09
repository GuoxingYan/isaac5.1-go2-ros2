"""Metric depth and MID360-like rays against the supplied collision mesh."""
import hashlib
from pathlib import Path

import numpy as np
import warp as wp
from scipy.spatial.transform import Rotation

from go2.sensor_profile import PROFILE, mid360_directions


@wp.kernel(enable_backward=False)
def _cast(mesh: wp.uint64, origins: wp.array(dtype=wp.vec3), directions: wp.array(dtype=wp.vec3), distances: wp.array(dtype=float)):
    index = wp.tid()
    t = float(0.0)
    u = float(0.0)
    v = float(0.0)
    sign = float(0.0)
    normal = wp.vec3()
    face = int(0)
    if wp.mesh_query_ray(mesh, origins[index], directions[index], 70.0, t, u, v, sign, normal, face):
        distances[index] = t
    else:
        distances[index] = -1.0


def _extract_usd_triangles(path):
    """Extract authored USD mesh triangles without changing scene coordinates."""
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.Open(str(path))
    if not stage:
        raise RuntimeError(f"Could not open collision USD: {path}")
    vertices = []
    faces = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get() or []
        indices = list(mesh.GetFaceVertexIndicesAttr().Get() or [])
        counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
        if not points or not indices or not counts:
            continue
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        start = len(vertices)
        for point in points:
            world = transform.Transform(Gf.Vec3d(float(point[0]), float(point[1]), float(point[2])))
            vertices.append([float(world[0]), float(world[1]), float(world[2])])
        cursor = 0
        for count in counts:
            polygon = indices[cursor:cursor + int(count)]
            cursor += int(count)
            for index in range(1, len(polygon) - 1):
                faces.append([start + polygon[0], start + polygon[index], start + polygon[index + 1]])
    if not vertices or not faces:
        raise RuntimeError(f"No mesh triangles found in collision USD: {path}")
    return np.asarray(vertices, dtype=np.float32), np.asarray(faces, dtype=np.int32)


def load_collision_mesh(path=None, cache_dir=None):
    """Load a hash-keyed triangle cache derived from the supplied collision USD."""
    source = Path(path) if path is not None else Path(__file__).resolve().parents[1] / "data" / "839873_collision.usd"
    source = source.resolve()
    if not source.exists():
        raise FileNotFoundError(f"Missing collision USD: {source}")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    cache_root = Path(cache_dir) if cache_dir is not None else Path(__file__).resolve().parents[1] / "outputs" / "mesh_cache"
    cache_root.mkdir(parents=True, exist_ok=True)
    cache = cache_root / f"{source.stem}_{digest[:16]}.npz"
    if not cache.exists():
        vertices, faces = _extract_usd_triangles(source)
        np.savez_compressed(cache, vertices=vertices, faces=faces, source_sha256=np.asarray(digest))
    values = np.load(cache)
    cached_hash = str(values["source_sha256"]) if "source_sha256" in values.files else ""
    if cached_hash and cached_hash != digest:
        raise RuntimeError(f"Mesh cache hash mismatch: {cache}")
    return wp.Mesh(
        points=wp.array(values["vertices"], dtype=wp.vec3, device="cuda:0"),
        indices=wp.array(values["faces"].ravel(), dtype=wp.int32, device="cuda:0"),
    )


def _raycast(mesh, origins, directions):
    distances = wp.empty(len(directions), dtype=float, device="cuda:0")
    wp.launch(
        _cast,
        dim=len(directions),
        inputs=[
            mesh.id,
            wp.array(origins, dtype=wp.vec3, device="cuda:0"),
            wp.array(directions, dtype=wp.vec3, device="cuda:0"),
            distances,
        ],
        device="cuda:0",
    )
    return distances.numpy()


class CollisionMeshLidar:
    """Cast the simulated MID360 angular coverage against real scene triangles."""

    def __init__(self, env, robot_index=0):
        self.env = env
        self.robot_index = robot_index
        self.source_path = Path(__file__).resolve().parents[1] / "data" / "839873_collision.usd"
        self.mesh = load_collision_mesh(self.source_path)
        self.last_tick = -1
        self.cached = {}
        self.scan_times = []

    def get_data(self):
        stamp = float(self.env.unwrapped.sim.current_time)
        tick = int(round(stamp * 1_000_000) * 10 // 1_000_000)
        if tick == self.last_tick:
            return self.cached
        self.last_tick = tick
        azimuth, elevation = mid360_directions(tick)
        az = np.radians(azimuth)
        el = np.radians(elevation)
        local = np.column_stack([
            np.cos(el) * np.cos(az),
            np.cos(el) * np.sin(az),
            np.sin(el),
        ]).astype(np.float32)
        state = self.env.unwrapped.scene["unitree_go2"].data.root_state_w[self.robot_index].detach().cpu().numpy()
        body_rotation = Rotation.from_quat(state[[4, 5, 6, 3]])
        lidar_rotation = body_rotation * Rotation.from_euler("xyz", PROFILE["lidar"]["rpy_deg"], degrees=True)
        origin = state[:3] + body_rotation.apply(PROFILE["lidar"]["position"])
        directions = lidar_rotation.apply(local).astype(np.float32)
        origins = np.tile(origin, (len(local), 1)).astype(np.float32)
        distances = _raycast(self.mesh, origins, directions)
        valid = (distances >= 0.1) & (distances <= 70.0) & np.isfinite(distances)
        ray_index = np.arange(len(local), dtype=np.int64)
        # The recorded PointCloud2 stores absolute nanoseconds. Keep a
        # deterministic 5 us firing offset for each simulated return.
        point_timestamps_ns = stamp * 1e9 + ray_index * 5000.0
        self.cached = {
            "data": local[valid] * distances[valid, None],
            "distance": distances[valid],
            "azimuth": azimuth[valid],
            "elevation": elevation[valid],
            "intensity": np.zeros(int(valid.sum()), dtype=np.float32),
            "timestamp": point_timestamps_ns[valid].astype(np.float64),
            "line": (ray_index[valid] % 4).astype(np.uint8),
            "tag": np.zeros(int(valid.sum()), dtype=np.uint8),
            "info": {
                "backend": "Warp CUDA collision triangle raycast",
                "scan_time": stamp,
                "scan_start_time": stamp,
                "scan_end_time": stamp + (len(local) - 1) * 5e-6,
                "rays": len(local),
                "returns": int(valid.sum()),
                "timestamp_unit": "nanoseconds since simulation clock",
                "instantaneous_scan": False,
                "source_mesh": str(self.source_path),
            },
        }
        self.scan_times.append(stamp)
        return self.cached


class CollisionMeshDepth:
    """Return axis-aligned metric depth from the same collision mesh."""

    def __init__(self, env, rig, mesh):
        self.env = env
        self.rig = rig
        self.mesh = mesh
        self.last_time = -1.0
        self.cached = None
        k = rig.get_depth_intrinsics_matrix()
        width, height = PROFILE["camera"]["depth_resolution"]
        v, u = np.indices((height, width))
        rays = np.column_stack([(u.ravel() - k[0, 2]) / k[0, 0], (v.ravel() - k[1, 2]) / k[1, 1], np.ones(width * height)])
        self.norm = np.linalg.norm(rays, axis=1).astype(np.float32)
        self.local = (rays / self.norm[:, None]).astype(np.float32)
        self.shape = (height, width)

    def get_depth(self):
        stamp = float(self.env.unwrapped.sim.current_time)
        if stamp == self.last_time:
            return self.cached.copy()
        self.last_time = stamp
        state = self.env.unwrapped.scene["unitree_go2"].data.root_state_w[0].detach().cpu().numpy()
        body_rotation = Rotation.from_quat(state[[4, 5, 6, 3]])
        rotation = body_rotation * Rotation.from_euler("xyz", PROFILE["camera"]["rpy_deg"], degrees=True)
        position = state[:3] + body_rotation.apply(PROFILE["camera"]["position"])
        origins = np.tile(position, (len(self.local), 1)).astype(np.float32)
        directions = rotation.apply(self.local).astype(np.float32)
        distance = _raycast(self.mesh, origins, directions)
        depth = distance / self.norm
        depth[(depth < 0.2) | (depth > 10.0) | ~np.isfinite(depth)] = np.nan
        self.cached = depth.reshape(self.shape)
        return self.cached.copy()
