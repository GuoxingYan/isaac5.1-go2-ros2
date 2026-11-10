import unittest

import numpy as np

from go2.path_cruise import ScenePathCruise


def path(points):
    return {"frame_id": "scene", "poses": [{"x": x, "y": y, "yaw": yaw} for x, y, yaw in points]}


class CruiseTests(unittest.TestCase):
    def test_short_segments_are_not_skipped_and_have_drive_command(self):
        cruise = ScenePathCruise(path([(0, 0, 0), (.15, 0, 1.57), (.30, .15, 0)]), loop=False)
        command = cruise.step([0, 0], 0, 0)
        self.assertEqual(cruise.index, 1)
        self.assertGreaterEqual(command[0], .26)
        self.assertAlmostEqual(command[2], 0.)  # current segment, not next-point yaw
        cruise.step([.14, 0], 0, .7)
        self.assertEqual(cruise.index, 2)

    def test_corner_turns_without_forward_motion(self):
        cruise = ScenePathCruise(path([(0, 0, 0), (1, 0, 0), (1, 1, 0)]), loop=False)
        cruise.step([.98, 0], 0, 1)
        self.assertEqual(cruise.index, 2)
        self.assertEqual(cruise.diagnostics["state"], "turning")
        self.assertEqual(cruise.command[0], 0.)
        self.assertGreater(cruise.command[2], 0.)

    def test_final_waypoint_requires_actual_return(self):
        cruise = ScenePathCruise(path([(0, 0, 0), (1, 0, 0), (0, 0, 0)]), loop=False)
        cruise.step([.95, 0], 0, 0)
        cruise.step([.3, 0], np.pi, 1)
        self.assertFalse(cruise.finished)
        cruise.step([.05, 0], np.pi, 2)
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
