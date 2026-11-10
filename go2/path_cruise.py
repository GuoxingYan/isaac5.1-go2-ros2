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
    """Track ordered segments; a fresh external ROS command retains priority.

    The map check uses only the robot center, not a collision footprint.  This
    controller is a simulator motion source, not an obstacle avoidance system.
    """

    def __init__(self, path_file, loop=True, position_tolerance=0.10,
                 clearance_check=None, required_clearance=0.2,
                 max_path_error=0.30, stall_seconds=8.0, stall_distance=0.10):
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
        self.index = 1
        self.completed_loops = 0
        self.finished = False
        self.aborted = False
        self.abort_reason = None
        self.command = np.zeros(3, dtype=np.float32)
        self.diagnostics = {}
        self._watch_time = None
        self._watch_xy = None
        self._watch_yaw = None

    def _safe_shortcut(self, start, end):
        if self.clearance_check is None:
            return True
        distance = float(np.linalg.norm(end - start))
        count = max(1, int(math.ceil(distance / 0.025)))
        return all(self.clearance_check(start + t * (end - start)) + 1e-9 >= self.required_clearance
                   for t in np.linspace(0., 1., count + 1))

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
        """Compute one bounded command from a truth pose and simulation time."""
        position = np.asarray(position, dtype=float)[:2]
        yaw = float(yaw)
        if self.finished or self.aborted:
            self.command[:] = 0.0
            return self.command.copy()
        if not np.isfinite(up_vector_z) or up_vector_z < 0.65:
            self._stop(f"robot tilted or fallen: body up z={up_vector_z:.3f}")
            self.diagnostics = {"state": "aborted", "body_up_z": float(up_vector_z)}
            return self.command.copy()

        # Intermediate waypoints may be passed only close to the segment.
        # Unlike radius-only advancement, this also detects an overshot point.
        for _ in range(len(self.waypoints)):
            start = self.waypoints[self.index - 1][:2]
            target = self.waypoints[self.index][:2]
            segment = target - start
            length = float(np.linalg.norm(segment))
            distance = float(np.linalg.norm(target - position))
            if length > 1e-6:
                progress = float(np.dot(position - start, segment) / (length * length))
                closest = start + np.clip(progress, 0., 1.) * segment
                path_error = float(np.linalg.norm(position - closest))
            else:
                progress, path_error = 1.0, distance
            final = self.index == len(self.waypoints) - 1 and not self.loop
            tolerance = self.final_tolerance if final else self.position_tolerance
            passed = not final and progress >= 1.0 and path_error <= self.position_tolerance
            advance = distance <= tolerance or passed
            if advance and not final:
                next_index = (self.index + 1) % len(self.waypoints)
                next_target = self.waypoints[next_index][:2]
                next_segment = next_target - target
                norm = np.linalg.norm(next_segment)
                lookahead = target + next_segment * min(1., .25 / max(norm, 1e-6))
                advance = self._safe_shortcut(position, lookahead)
            if advance:
                self._advance()
                if self.finished:
                    self.command[:] = 0.0
                    self.diagnostics = {"state": "finished", "distance_to_goal_m": distance}
                    return self.command.copy()
                continue
            break
        else:
            self._stop("all waypoints were exhausted without a usable segment")
            return self.command.copy()

        clearance = None if self.clearance_check is None else self.clearance_check(position)
        if clearance is not None and clearance + 1e-9 < self.required_clearance:
            self._stop(f"robot center clearance {clearance:.3f} m < {self.required_clearance:.3f} m")
        elif path_error > self.max_path_error:
            self._stop(f"segment deviation {path_error:.3f} m > {self.max_path_error:.3f} m")
        if self.aborted:
            self.diagnostics = {"state": "aborted", "clearance_m": clearance, "path_error_m": path_error}
            return self.command.copy()

        # Aim heading at a bounded lookahead on this segment so lateral
        # drift is corrected by steering, rather than relying on weak vy alone.
        along = np.clip(progress * length + 0.75, 0., length)
        heading_point = start + (along / max(length, 1e-6)) * segment
        heading_vector = heading_point - position
        desired_yaw = math.atan2(heading_vector[1], heading_vector[0])
        yaw_error = _angle_delta(desired_yaw - yaw)
        # Point at the current segment, never at the following waypoint's yaw.
        # Rotate in place at large corners instead of cutting across obstacles.
        turning = abs(yaw_error) > 0.65
        wz = float(np.clip(1.5 * yaw_error, -0.6, 0.6))
        if turning:
            vx = vy = 0.0
        else:
            forward = math.cos(yaw) * (target[0] - position[0]) + math.sin(yaw) * (target[1] - position[1])
            lateral = -math.sin(yaw) * (target[0] - position[0]) + math.cos(yaw) * (target[1] - position[1])
            # Avoid the old double attenuation near densely spaced points.
            vx = float(np.clip(0.8 * forward, -0.32, 0.32))
            if forward > 0.03 and distance > tolerance:
                vx = max(vx, 0.27)
            vy = float(np.clip(0.7 * lateral, -0.10, 0.10))
        self.command[:] = (vx, vy, wz)
        self.diagnostics = {"state": "turning" if turning else "tracking", "target_distance_m": distance,
                            "path_error_m": path_error, "clearance_m": clearance,
                            "yaw_error_rad": yaw_error, "segment_progress": progress}

        if self._watch_time is None or sim_time < self._watch_time:
            self._watch_time, self._watch_xy, self._watch_yaw = float(sim_time), position.copy(), yaw
        elif sim_time - self._watch_time >= self.stall_seconds:
            motion = float(np.linalg.norm(position - self._watch_xy))
            rotation = abs(_angle_delta(yaw - self._watch_yaw))
            self._watch_time, self._watch_xy, self._watch_yaw = float(sim_time), position.copy(), yaw
            if motion < self.stall_distance and rotation < 0.30:
                self._stop(f"no progress for {self.stall_seconds:.1f} sim s: moved {motion:.3f} m, rotated {rotation:.3f} rad")
                self.diagnostics["state"] = "aborted"
        return self.command.copy()

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
