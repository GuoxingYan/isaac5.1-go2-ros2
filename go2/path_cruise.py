"""Small scene-frame waypoint controller used when no navigation command is present."""
import json
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation


class ScenePathCruise:
    """Follow a checked scene-frame path with the frozen Go2 velocity policy.

    This is only a simulator cruise source. A fresh /cmd_vel2 command takes
    precedence, so the same run can later be driven by the real navigation
    stack without changing the sensor or bridge path.
    """

    def __init__(self, path_file, loop=True, position_tolerance=0.22):
        if isinstance(path_file, dict):
            self.path_file = "generated from occupancy map"
            data = path_file
        else:
            self.path_file = str(Path(path_file).resolve())
            data = json.loads(Path(self.path_file).read_text())
        self.frame_id = data.get("frame_id", "scene")
        if self.frame_id != "scene":
            raise ValueError(f"InteriorGS path must use frame_id=scene, got {self.frame_id!r}")
        self.waypoints = [
            np.asarray([float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0))])
            for p in data.get("poses", [])
        ]
        if len(self.waypoints) < 2:
            raise ValueError(f"Scene path needs at least two poses: {self.path_file}")
        self.loop = bool(loop)
        self.position_tolerance = float(position_tolerance)
        self.index = 1
        self.completed_loops = 0
        self.command = np.zeros(3, dtype=np.float32)

    @property
    def start_xy(self):
        return self.waypoints[0][:2].copy()

    @property
    def start_yaw(self):
        return float(self.waypoints[0][2])

    def update(self, env, ctrl):
        """Update the policy velocity command from the current base pose."""
        # Navigation commands received through the bridge have priority.
        if ctrl.ros_commands:
            return

        state = env.unwrapped.scene["unitree_go2"].data.root_state_w[0]
        state = state.detach().cpu().numpy()
        position = state[:2]
        yaw = Rotation.from_quat(state[[4, 5, 6, 3]]).as_euler("zyx")[0]
        target = self.waypoints[self.index]
        delta_world = target[:2] - position
        distance = float(np.linalg.norm(delta_world))

        if distance < self.position_tolerance:
            if self.index == len(self.waypoints) - 1:
                if not self.loop:
                    ctrl.base_vel_cmd_input[0].zero_()
                    self.command[:] = 0.0
                    return
                self.index = 0
                self.completed_loops += 1
            else:
                self.index += 1
            target = self.waypoints[self.index]
            delta_world = target[:2] - position
            distance = float(np.linalg.norm(delta_world))

        # Convert the waypoint error into the policy's body velocity command.
        c, s = np.cos(yaw), np.sin(yaw)
        forward = c * delta_world[0] + s * delta_world[1]
        lateral = -s * delta_world[0] + c * delta_world[1]
        desired_yaw = float(target[2])
        yaw_error = np.arctan2(np.sin(desired_yaw - yaw), np.cos(desired_yaw - yaw))
        speed_scale = min(1.0, distance / 0.8)
        vx = np.clip(0.7 * forward, -0.4, 0.4) * speed_scale
        vy = np.clip(0.8 * lateral, -0.1, 0.1) * speed_scale
        wz = np.clip(1.2 * yaw_error, -0.6, 0.6)
        self.command[:] = (vx, vy, wz)
        ctrl.base_vel_cmd_input[0] = torch.as_tensor(
            self.command, dtype=ctrl.base_vel_cmd_input.dtype,
            device=ctrl.base_vel_cmd_input.device,
        )
