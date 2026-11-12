"""Conservative scene-frame path tracking for the frozen Go2 velocity policy."""
import json
import math
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation


def _angle_delta(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ScenePathCruise:
    """Rotate in place to each segment, then drive forward only.

    The map check uses only the robot center, not a collision footprint.  This
    controller is a simulator motion source, not an obstacle avoidance system.
    """

    def __init__(self, path_file, loop=True, position_tolerance=0.10,
                 clearance_check=None, required_clearance=0.2,
                 max_path_error=0.30, stall_seconds=8.0, stall_distance=0.10,
                 corner_slowdown_distance=0.50, yaw_tolerance=0.12,
                 min_forward_speed=0.27, yaw_realign_tolerance=0.25):
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
        self.final_tolerance = 0.12
        self.clearance_check = clearance_check
        self.required_clearance = float(required_clearance)
        self.max_path_error = float(max_path_error)
        self.stall_seconds = float(stall_seconds)
        self.stall_distance = float(stall_distance)
        self.corner_slowdown_distance = float(corner_slowdown_distance)
        self.yaw_tolerance = float(yaw_tolerance)
        self.min_forward_speed = float(min_forward_speed)
        self.yaw_realign_tolerance = float(yaw_realign_tolerance)
        self.index = 1
        self.completed_loops = 0
        self.finished = False
        self.aborted = False
        self.abort_reason = None
        self.command = np.zeros(3, dtype=np.float32)
        self.diagnostics = {}
        self.phase = "turn"
        self._watch_time = None
        self._watch_xy = None
        self._watch_yaw = None

    @property
    def start_xy(self):
        return self.waypoints[0][:2].copy()

    @property
    def start_yaw(self):
        return float(self.waypoints[0][2])

    def _stop(self, reason):
        self.aborted = True
        self.abort_reason = reason
        self.command[:] = 0.0
        print(f"[CRUISE ABORT] {reason}", flush=True)

    def _advance(self):
        if self.index == len(self.waypoints) - 1:
            if not self.loop:
                self.finished = True
                return
            self.index = 0
            self.completed_loops += 1
        else:
            self.index += 1

    def step(self, position, yaw, sim_time, up_vector_z=1.0):
        """Compute a mutually exclusive yaw or forward command."""
        position = np.asarray(position, dtype=float)[:2]
        yaw = float(yaw)
        if self.finished or self.aborted:
            self.command[:] = 0.0
            return self.command.copy()
        if not np.isfinite(up_vector_z) or up_vector_z < 0.65:
            self._stop(f"robot tilted or fallen: body up z={up_vector_z:.3f}")
            self.diagnostics = {"state": "aborted", "body_up_z": float(up_vector_z)}
            return self.command.copy()

        target = self.waypoints[self.index][:2]
        final = self.index == len(self.waypoints) - 1 and not self.loop
        tolerance = self.final_tolerance if final else self.position_tolerance
        start = self.waypoints[self.index - 1][:2]
        segment = target - start
        length = float(np.linalg.norm(segment))
        if length <= 1e-6:
            self._stop(f"waypoint segment {self.index} has zero length")
            return self.command.copy()

        if self.phase == "drive" and np.linalg.norm(target - position) <= tolerance:
            if final:
                self.finished = True
                self.command[:] = 0.0
                self.diagnostics = {"state": "finished", "distance_to_goal_m": float(np.linalg.norm(target-position))}
                return self.command.copy()
            self._advance()
            self.phase = "turn"
            target = self.waypoints[self.index][:2]
            start = self.waypoints[self.index - 1][:2]
            segment = target - start
            length = float(np.linalg.norm(segment))

        distance = float(np.linalg.norm(target - position))
        progress = float(np.clip(np.dot(position - start, segment) / (length * length), 0., 1.))
        closest = start + progress * segment
        path_error = float(np.linalg.norm(position - closest))
        clearance = None if self.clearance_check is None else self.clearance_check(position)
        if clearance is not None and clearance + 1e-9 < self.required_clearance:
            self._stop(f"robot center clearance {clearance:.3f} m < {self.required_clearance:.3f} m")
        elif path_error > self.max_path_error:
            self._stop(f"segment deviation {path_error:.3f} m > {self.max_path_error:.3f} m")
        if self.aborted:
            self.diagnostics = {"state": "aborted", "clearance_m": clearance, "path_error_m": path_error}
            return self.command.copy()

        desired_yaw = math.atan2(segment[1], segment[0])
        yaw_error = _angle_delta(desired_yaw - yaw)
        if self.phase == "drive" and abs(yaw_error) > self.yaw_realign_tolerance:
            self.phase = "turn"
        if self.phase == "turn":
            if abs(yaw_error) <= self.yaw_tolerance:
                self.phase = "drive"
            else:
                wz = float(np.clip(1.5 * yaw_error, -0.6, 0.6))
                if abs(wz) < 0.20:
                    wz = math.copysign(0.20, yaw_error)
                self.command[:] = (0.0, 0.0, wz)
                self.diagnostics = {"state": "turning_to_segment", "phase": self.phase,
                                    "target_distance_m": distance, "path_error_m": path_error,
                                    "clearance_m": clearance, "yaw_error_rad": yaw_error,
                                    "speed_scale": 0.0}
                self._check_stall(position, yaw, sim_time)
                return self.command.copy()

        speed_scale = min(1.0, max(0.0, distance / max(self.corner_slowdown_distance, 1e-6)))
        vx = max(self.min_forward_speed, 0.32 * speed_scale) if distance > tolerance else 0.0
        self.command[:] = (vx, 0.0, 0.0)
        self.diagnostics = {"state": "approaching_waypoint" if distance < self.corner_slowdown_distance else "driving",
                            "phase": self.phase, "target_distance_m": distance,
                            "path_error_m": path_error, "clearance_m": clearance,
                            "yaw_error_rad": yaw_error, "segment_progress": progress,
                            "speed_scale": speed_scale}

        self._check_stall(position, yaw, sim_time)
        return self.command.copy()

    def _check_stall(self, position, yaw, sim_time):
        if self._watch_time is None or sim_time < self._watch_time:
            self._watch_time, self._watch_xy, self._watch_yaw = float(sim_time), position.copy(), yaw
        elif sim_time - self._watch_time >= self.stall_seconds:
            motion = float(np.linalg.norm(position - self._watch_xy))
            rotation = abs(_angle_delta(yaw - self._watch_yaw))
            self._watch_time, self._watch_xy, self._watch_yaw = float(sim_time), position.copy(), yaw
            if motion < self.stall_distance and rotation < 0.30:
                self._stop(f"no progress for {self.stall_seconds:.1f} sim s: moved {motion:.3f} m, rotated {rotation:.3f} rad")
                self.diagnostics["state"] = "aborted"

    def update(self, env, ctrl):
        if ctrl.ros_commands:
            self._watch_time = None
            return
        state = env.unwrapped.scene["unitree_go2"].data.root_state_w[0].detach().cpu().numpy()
        rotation = Rotation.from_quat(state[[4, 5, 6, 3]])
        yaw = rotation.as_euler("zyx")[0]
        command = self.step(state[:2], yaw, float(env.unwrapped.sim.current_time), rotation.as_matrix()[2, 2])
        ctrl.base_vel_cmd_input[0] = torch.as_tensor(
            command, dtype=ctrl.base_vel_cmd_input.dtype, device=ctrl.base_vel_cmd_input.device,
        )
