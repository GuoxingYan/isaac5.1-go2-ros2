import unittest

import numpy as np

from go2.path_cruise import ScenePathCruise


def path(points):
    return {"frame_id": "scene", "poses": [{"x": x, "y": y, "yaw": yaw} for x, y, yaw in points]}


class CruiseTests(unittest.TestCase):
    def assert_single_axis_command(self, command):
        self.assertLessEqual(np.count_nonzero(np.abs(command) > 1e-8), 1)

    def test_initial_heading_turns_in_place_then_drives_forward(self):
        cruise = ScenePathCruise(path([(0, 0, np.pi / 2), (1, 0, 0), (1, 1, 0)]), loop=False)
        command = cruise.step([0, 0], np.pi / 2, 0)
        self.assertEqual(cruise.diagnostics["state"], "turning_to_segment")
        self.assertEqual(command[0], 0.)
        self.assertEqual(command[1], 0.)
        self.assertLess(command[2], 0.)
        self.assert_single_axis_command(command)

        command = cruise.step([0, 0], 0, .5)
        self.assertEqual(cruise.diagnostics["state"], "driving")
        self.assertGreater(command[0], 0.)
        self.assertEqual(command[1], 0.)
        self.assertEqual(command[2], 0.)
        self.assert_single_axis_command(command)

    def test_approach_slowdown_and_corner_turn_are_axis_exclusive(self):
        cruise = ScenePathCruise(path([(0, 0, 0), (1, 0, 0), (1, 1, 0)]), loop=False)
        cruise.step([0, 0], 0, 0)
        fast = cruise.command[0]
        slow = cruise.step([.6, 0], 0, 1)
        self.assertEqual(cruise.diagnostics["state"], "approaching_waypoint")
        self.assertGreater(slow[0], 0.)
        self.assertLess(slow[0], fast)
        self.assertEqual(slow[1], 0.)
        self.assertEqual(slow[2], 0.)
        self.assert_single_axis_command(slow)

        command = cruise.step([.91, 0], 0, 1.1)
        self.assertEqual(cruise.index, 2)
        self.assertEqual(cruise.diagnostics["state"], "turning_to_segment")
        self.assertEqual(command[0], 0.)
        self.assertEqual(command[1], 0.)
        self.assertGreater(command[2], 0.)
        self.assert_single_axis_command(command)

        command = cruise.step([.91, 0], np.pi / 2, 1.2)
        self.assertEqual(cruise.index, 2)
        self.assertAlmostEqual(command[0], .32)
        self.assertEqual(command[1], 0.)
        self.assertEqual(command[2], 0.)
        self.assert_single_axis_command(command)

    def test_turn_phase_must_align_before_forward_motion(self):
        cruise = ScenePathCruise(path([(0, 0, 0), (1, 0, 0)]), loop=False)
        turn = cruise.step([0, 0], -np.pi / 2, 0)
        self.assertEqual(turn[0], 0.)
        self.assertEqual(turn[1], 0.)
        self.assertNotEqual(turn[2], 0.)
        self.assert_single_axis_command(turn)
        drive = cruise.step([0, 0], 0, .1)
        self.assertAlmostEqual(drive[0], .32)
        self.assertEqual(drive[1], 0.)
        self.assertEqual(drive[2], 0.)
        self.assert_single_axis_command(drive)

    def test_final_waypoint_requires_actual_return(self):
        cruise = ScenePathCruise(path([(0, 0, 0), (1, 0, 0), (0, 0, 0)]), loop=False)
        cruise.step([0, 0], 0, 0)
        cruise.step([.95, 0], 0, 1)
        reverse_turn = cruise.step([.95, 0], 0, 1.1)
        self.assertNotEqual(reverse_turn[2], 0.)
        cruise.step([.95, 0], np.pi, 1.2)
        cruise.step([.3, 0], np.pi, 2)
        self.assertFalse(cruise.finished)
        cruise.step([.05, 0], np.pi, 2.5)
        self.assertTrue(cruise.finished)
        np.testing.assert_array_equal(cruise.command, np.zeros(3))

    def test_stall_deviation_and_clearance_stop(self):
        basic = path([(0, 0, 0), (2, 0, 0)])
        stalled = ScenePathCruise(basic, loop=False)
        stalled.step([0, 0], 0, 0)
        stalled.step([.01, 0], 0, 8.1)
        self.assertTrue(stalled.aborted)
        self.assertIn("no progress", stalled.abort_reason)
        deviated = ScenePathCruise(basic, loop=False)
        deviated.step([0, .31], 0, 0)
        self.assertIn("deviation", deviated.abort_reason)
        unsafe = ScenePathCruise(basic, loop=False, clearance_check=lambda _: .19)
        unsafe.step([0, 0], 0, 0)
        self.assertIn("clearance", unsafe.abort_reason)
        fallen = ScenePathCruise(basic, loop=False)
        fallen.step([0, 0], 0, 0, up_vector_z=.3)
        self.assertIn("fallen", fallen.abort_reason)


if __name__ == "__main__":
    unittest.main()
