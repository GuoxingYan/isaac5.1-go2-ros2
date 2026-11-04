import omni
import numpy as np
from pxr import Gf

class SensorManager:
    def __init__(self, num_envs):
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.sensors.camera")
        enable_extension("isaacsim.sensors.rtx")
        import carb
        carb.settings.get_settings().set_bool("/isaaclab/render/rtx_sensors", True)
        self.render_products = []
        self.num_envs = num_envs

    def add_rtx_lidar(self):
        import omni.replicator.core as rep
        lidar_annotators = []
        for env_idx in range(self.num_envs):
            try:
                _, sensor = omni.kit.commands.execute(
                    "IsaacSensorCreateRtxLidar",
                    path="/lidar",
                    parent=f"/World/envs/env_{env_idx}/Go2/base",
                    config="Example_Rotary",
                    translation=(0.2, 0, 0.2),
                    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
                )
                try:
                    annotator = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
                    hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
                    self.render_products.append(hydra_texture)
                    annotator.initialize(transformPoints=False)
                    annotator.attach(hydra_texture.path)
                    lidar_annotators.append(annotator)
                except Exception as e:
                    print(f"[WARN] LiDAR annotator for env {env_idx} failed: {e}")
            except Exception as e:
                print(f"[WARN] LiDAR sensor creation for env {env_idx} failed: {e}")
        return lidar_annotators

    def add_camera(self, freq):
        try:
            from isaacsim.sensors.camera import Camera
            import isaacsim.core.utils.numpy.rotations as rot_utils
        except ImportError as e:
            print(f"[WARN] Camera import failed: {e}")
            return []
        cameras = []
        for env_idx in range(self.num_envs):
            try:
                camera = Camera(
                    prim_path=f"/World/envs/env_{env_idx}/Go2/base/front_cam",
                    translation=np.array([0.4, 0.0, 0.2]),
                    frequency=None,
                    resolution=(640, 480),
                    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
                )
                camera.initialize()
                camera.set_focal_length(0.018)
                camera.set_horizontal_aperture(0.024)
                camera.set_clipping_range(0.05, 100.0)
                camera.add_distance_to_image_plane_to_frame()
                camera.add_semantic_segmentation_to_frame()
                camera.add_pointcloud_to_frame(include_unlabelled=True)
                cameras.append(camera)
            except Exception as e:
                print(f"[WARN] Camera creation for env {env_idx} failed: {e}")
        return cameras