"""Scene setup for the supplied 839873 Gaussian and collision assets."""
import json
import hashlib
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt


class InteriorGSScene:
    """Keep the visual and metric assets in one explicit scene coordinate frame."""

    def __init__(self, data_dir, seed=None, clearance=0.45, spawn_xy=None, spawn_yaw=None):
        root = Path(data_dir).resolve()
        self.data_dir = root
        # The four files in data/ are one coordinate-consistent scene. Do not
        # silently fall back to an older recentered/source_frame_backup copy.
        self.asset_dir = root
        self.gaussian_path = root / "839873.usdz"
        self.collision_source_path = root / "839873_collision.usd"
        # The supplied collision USD is already in the scene frame. Loading it
        # directly avoids losing PhysX collision registration through an extra
        # reference wrapper.
        self.collision_path = self.collision_source_path
        if not self.gaussian_path.exists() or not self.collision_source_path.exists():
            raise FileNotFoundError(f"InteriorGS assets missing under {self.asset_dir}")
        occupancy_path = root / "occupancy.png"
        metadata_path = root / "occupancy.json"
        # The supplied archive currently contains a filename with a space. It
        # is accepted as a compatibility fallback, but no alternate geometry
        # or coordinate transform is applied.
        if not metadata_path.exists():
            metadata_path = root / "occupancy .json"
        if not occupancy_path.exists():
            raise FileNotFoundError(f"Missing occupancy image: {occupancy_path}")
        self.image = np.asarray(Image.open(occupancy_path).convert("L"))
        self.meta = self._read_json(metadata_path, {})
        if not self.meta:
            raise FileNotFoundError(f"Missing occupancy metadata: {root / 'occupancy.json'}")
        self.scale = float(self.meta["scale"])
        self.origin = np.asarray(self.meta.get("min", [-0.12, -11.44])[:2], dtype=float)
        self.free = self.image == 255
        self.clearance = distance_transform_edt(self.free) * self.scale
        self.required_clearance = float(clearance)
        self.seed = int(np.random.SeedSequence().entropy) if seed is None else int(seed)
        rng = np.random.default_rng(self.seed)
        self.spawn = None
        if spawn_xy is not None:
            self.spawn = self._make_spawn(np.asarray(spawn_xy, dtype=float), float(spawn_yaw or 0.0), "path_start")
            if self.spawn is None:
                raise ValueError(
                    f"Requested path start {spawn_xy} is blocked or below "
                    f"{self.required_clearance:.2f} m occupancy clearance"
                )
        else:
            candidates = np.argwhere(self.clearance >= self.required_clearance)
            if not len(candidates):
                raise RuntimeError(f"No occupancy-safe spawn with clearance {clearance}")
            row, col = candidates[rng.integers(len(candidates))]
            self.spawn = self._make_spawn(self.pixel_to_world(row, col), rng.uniform(-np.pi, np.pi), "safe_random")
        self.renderer = None
        self.mesh_support = {}
        self.evidence = {
            "scene_id": "839873",
            "coordinate_frame": "scene",
            "gaussian_source": str(self.gaussian_path),
            "collision_source": str(self.collision_source_path),
            "collision_runtime_asset": str(self.collision_path),
            "occupancy_image": str(occupancy_path),
            "occupancy_metadata": str(metadata_path),
            "asset_sha256": {
                "gaussian": hashlib.sha256(self.gaussian_path.read_bytes()).hexdigest(),
                "collision": hashlib.sha256(self.collision_source_path.read_bytes()).hexdigest(),
            },
            "asset_transform": "identity; source data/ coordinates are used as authored",
            "synthetic_ground": False,
            "spawn": self.spawn,
        }

    @staticmethod
    def _read_json(path, default):
        try:
            return json.loads(path.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            return default

    @staticmethod
    def path_start_pose(path_data):
        """Read a scene path's first planar pose, including ROS quaternions."""
        if path_data.get("frame_id", "scene") != "scene":
            raise ValueError("InteriorGS spawn path must use frame_id=scene")
        poses = path_data.get("poses")
        if not poses:
            raise ValueError("InteriorGS spawn path contains no poses")
        first = poses[0]
        xy = (float(first["x"]), float(first["y"]))
        if not all(math.isfinite(value) for value in xy):
            raise ValueError("Spawn position must be finite")
        quaternion_keys = ("qx", "qy", "qz", "qw")
        if any(key in first for key in quaternion_keys):
            if not all(key in first for key in quaternion_keys):
                raise ValueError("Spawn quaternion requires qx, qy, qz and qw")
            q = np.asarray([float(first[key]) for key in quaternion_keys])
            norm = float(np.linalg.norm(q))
            if not np.isfinite(norm) or norm < 1e-12:
                raise ValueError("Spawn quaternion must be finite and nonzero")
            x, y, z, w = q / norm
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        else:
            yaw = float(first.get("yaw", 0.0))
        if not math.isfinite(yaw):
            raise ValueError("Spawn yaw must be finite")
        return xy, yaw

    def _make_spawn(self, xy, yaw, mode):
        row, col = self.world_to_pixel(*xy)
        if not (0 <= row < self.image.shape[0] and 0 <= col < self.image.shape[1]):
            return None
        if self.image[row, col] != 255:
            return None
        if self.clearance[row, col] < self.required_clearance:
            return None
        return {
            "mode": mode,
            "seed": self.seed,
            "pixel_row_col": [int(row), int(col)],
            "position": [float(xy[0]), float(xy[1]), 0.42],
            "yaw_rad": float(yaw),
            "clearance_m": float(self.clearance[row, col]),
            "required_clearance_m": self.required_clearance,
            "occupancy_value": int(self.image[row, col]),
            "mapping": {"scale": self.scale, "origin_xy": self.origin.tolist()},
        }

    def pixel_to_world(self, row, col):
        x = self.origin[0] + (self.image.shape[1] - col - 0.5) * self.scale
        y = self.origin[1] + (row + 0.5) * self.scale
        return np.array([x, y])

    def world_to_pixel(self, x, y):
        col = int(np.floor(self.image.shape[1] - (x - self.origin[0]) / self.scale))
        row = int(np.floor((y - self.origin[1]) / self.scale))
        return row, col

    def center_clearance(self, xy):
        row, col = self.world_to_pixel(*xy)
        if not (0 <= row < self.image.shape[0] and 0 <= col < self.image.shape[1]):
            return 0.0
        return float(self.clearance[row, col])

    @classmethod
    def default_path(cls, data_dir, clearance=0.45):
        """Build a deterministic safe route from the supplied occupancy map.

        This route is only a simulator motion source. It is not a localization
        result and it is not published as the navigation module's map path.
        """
        root = Path(data_dir).resolve()
        image_path = root / "occupancy.png"
        metadata_path = root / "occupancy.json"
        if not metadata_path.exists():
            metadata_path = root / "occupancy .json"
        image = np.asarray(Image.open(image_path).convert("L"))
        meta = cls._read_json(metadata_path, {})
        if not meta:
            raise FileNotFoundError(f"Missing occupancy metadata: {metadata_path}")
        scale = float(meta["scale"])
        origin = np.asarray(meta["min"][:2], dtype=float)
        from env.occupancy_loop import plan_loop
        return plan_loop(image, scale, origin, clearance)

    def validate_path(self, path_data, sample_step=0.025):
        """Reject waypoints or segments that leave the clearance-checked floor."""
        clearance_required = float(path_data.get("planning", {}).get(
            "minimum_clearance_m", self.required_clearance))
        poses = path_data.get("poses", [])
        if path_data.get("frame_id", "scene") != "scene" or len(poses) < 2:
            raise ValueError("InteriorGS path must have at least two scene-frame poses")
        points = np.asarray([[float(p["x"]), float(p["y"])] for p in poses], dtype=float)
        for index, point in enumerate(points):
            row, col = self.world_to_pixel(*point)
            if not (0 <= row < self.image.shape[0] and 0 <= col < self.image.shape[1]):
                raise ValueError(f"Path waypoint {index} is outside occupancy: {point.tolist()}")
            if self.clearance[row, col] + 1e-9 < clearance_required:
                raise ValueError(f"Path waypoint {index} lacks clearance: {point.tolist()}")
        endpoints = points[1:]
        if path_data.get("loop", False):
            endpoints = np.vstack([endpoints, points[:1]])
        for index, (start, end) in enumerate(zip(points[:len(endpoints)], endpoints)):
            count = max(1, int(np.ceil(np.linalg.norm(end - start) / sample_step)))
            for fraction in np.linspace(0.0, 1.0, count + 1):
                point = start + fraction * (end - start)
                row, col = self.world_to_pixel(*point)
                if not (0 <= row < self.image.shape[0] and 0 <= col < self.image.shape[1]):
                    raise ValueError(f"Path segment {index} leaves occupancy: {point.tolist()}")
                if self.clearance[row, col] + 1e-9 < clearance_required:
                    raise ValueError(f"Path segment {index} lacks clearance: {point.tolist()}")

    @staticmethod
    def minimum_area_rectangle_path(path_data, clearance=0.15):
        """Reduce route nodes to the four corners of their minimum-area rectangle."""
        import cv2

        poses = path_data.get("poses", [])
        points = np.asarray([[float(p["x"]), float(p["y"])] for p in poses], dtype=np.float32)
        if len(points) < 3:
            raise ValueError("At least three route nodes are needed for a bounding rectangle")
        corners = cv2.boxPoints(cv2.minAreaRect(points)).astype(np.float64)
        result = []
        for index, (x, y) in enumerate(corners):
            next_point = corners[(index + 1) % len(corners)]
            delta = next_point - corners[index]
            result.append({"x": float(x), "y": float(y), "yaw": float(np.arctan2(delta[1], delta[0]))})
        return {
            "frame_id": path_data.get("frame_id", "scene"),
            "loop": True,
            "source": "minimum-area rectangle of scene route nodes",
            "poses": result,
            "planning": {
                "minimum_clearance_m": float(clearance),
                "source_waypoint_count": len(poses),
                "route_length_m": float(sum(
                    np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)
                )),
            },
        }

    def configure(self, cfg):
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg
        from scipy.spatial.transform import Rotation

        cfg.sim.enable_scene_query_support = True
        cfg.scene.ground = None
        cfg.scene.height_scanner = None
        cfg.observations.policy.height_scan = None
        cfg.scene.interior_mesh = AssetBaseCfg(
            prim_path="/World/InteriorMesh",
            spawn=sim_utils.UsdFileCfg(usd_path=str(self.collision_path)),
        )
        cfg.scene.unitree_go2.init_state.pos = tuple(self.spawn["position"])
        quaternion = Rotation.from_euler("z", self.spawn["yaw_rad"]).as_quat()
        cfg.scene.unitree_go2.init_state.rot = tuple(quaternion[[3, 0, 1, 2]].tolist())
        cfg.viewer.eye = np.asarray(self.spawn["position"]) + np.asarray([-0.6, 0.8, 1.8])
        cfg.viewer.lookat = self.spawn["position"]

    def initialize(self, env, cameras, lidars, show_rgb=True):
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics
        from env.mesh_lidar import CollisionMeshDepth, load_collision_mesh

        stage = omni.usd.get_context().get_stage()
        mesh_prim = stage.GetPrimAtPath("/World/InteriorMesh")
        for prim in Usd.PrimRange(mesh_prim):
            if prim.IsA(UsdGeom.Mesh):
                UsdGeom.Mesh(prim).GetDoubleSidedAttr().Set(True)
        self.mesh_count = sum(prim.IsA(UsdGeom.Mesh) for prim in Usd.PrimRange(mesh_prim))
        # Keep the user asset unchanged, but make the runtime contract
        # explicit for assets whose authored collision schemas are sparse.
        collision_prims = 0
        mesh_collision_prims = 0
        for prim in Usd.PrimRange(mesh_prim):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                UsdPhysics.MeshCollisionAPI.Apply(prim)
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(True)
            UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Set("none")
            collision_prims += int(prim.HasAPI(UsdPhysics.CollisionAPI))
            mesh_collision_prims += int(prim.HasAPI(UsdPhysics.MeshCollisionAPI))
        if stage.GetPrimAtPath("/World/ground").IsValid() or stage.GetPrimAtPath("/World/obstacleTerrain").IsValid():
            raise RuntimeError("Synthetic ground/obstacle terrain is present in InteriorGS mode")

        # PhysX scene queries can run before the referenced static mesh has
        # finished cooking. Validate support against the same authored USD
        # triangles used by the LiDAR instead of treating that transient query
        # result as evidence of a missing floor.
        metric_mesh = lidars[0].mesh if lidars else load_collision_mesh(self.collision_path)
        from env.mesh_lidar import _raycast
        x, y, _ = self.spawn["position"]
        support_distance = _raycast(
            metric_mesh,
            np.asarray([[x, y, 1.0]], dtype=np.float32),
            np.asarray([[0.0, 0.0, -1.0]], dtype=np.float32),
        )[0]
        if not np.isfinite(support_distance) or support_distance < 0.0:
            raise RuntimeError("Spawn has no support hit on 839873_collision.usd triangles")
        position = (x, y, 1.0 - float(support_distance))
        self.mesh_support = {
            "hit": True,
            "position": [float(v) for v in position],
            "normal": [0.0, 0.0, 1.0],
            "distance": float(support_distance),
            "collision": "authored 839873_collision.usd triangle query",
            "rigidBody": "static mesh asset",
            "physx_scene_query": "not used for readiness; USD mesh query is deterministic",
        }
        if abs(self.mesh_support["position"][2]) > 0.15:
            raise RuntimeError(f"Spawn support height invalid: {self.mesh_support}")

        for rig in cameras:
            if self.renderer is None:
                from env.gaussian_rgb import GaussianRGBRenderer
                self.renderer = GaussianRGBRenderer(self.gaussian_path, [0.0, 0.0, 0.0])
            rig.gaussian_renderer = self.renderer
            rig.depth_provider = CollisionMeshDepth(env, rig, metric_mesh)
        self.evidence.update({
            "mesh_count": self.mesh_count,
            "collision_api_meshes": collision_prims,
            "mesh_collision_api_meshes": mesh_collision_prims,
            "support": self.mesh_support,
            "rgb_source": "839873.usdz NuRec Gaussians" if cameras else "disabled for mapping run",
            "metric_source": "839873_collision.usd triangles",
            "coordinate_frame": "data/ source frame; no recentering",
            "renderer": self.renderer.info if getattr(self, "renderer", None) is not None else None,
        })

        if show_rgb and cameras:
            import omni.ui as ui

            self.window = ui.Window("D435 RGB | 839873 Gaussian", width=660, height=410)
            self.provider = ui.ByteImageProvider()
            with self.window.frame:
                with ui.VStack():
                    ui.Label("RGB: 839873.usdz Gaussian | depth/LiDAR: collision USD", height=24)
                    ui.ImageWithProvider(self.provider, height=360)
            cameras[0].rgb_display = self.provider

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        if self.renderer is not None and self.renderer.last_image is not None:
            Image.fromarray(self.renderer.last_image).save(directory / "scene_rgb.png")
        self.evidence["last_camera_pose"] = self.renderer.last_pose if self.renderer else None
        self.evidence["rendered_frames"] = self.renderer.frames if self.renderer else 0
        self.evidence["mean_gaussian_render_s"] = float(np.mean(self.renderer.timings)) if self.renderer and self.renderer.timings else 0.0
        (directory / "scene_evidence.json").write_text(json.dumps(self.evidence, indent=2))
