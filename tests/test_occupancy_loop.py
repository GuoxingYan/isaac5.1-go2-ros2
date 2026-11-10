import unittest
import numpy as np
from PIL import Image
from env.occupancy_loop import plan_loop
from env.interiorgs_scene import InteriorGSScene

class LoopTests(unittest.TestCase):
    def test_scene_loop(self):
        path = InteriorGSScene.default_path('data', .2)
        again = InteriorGSScene.default_path('data', .2)
        self.assertEqual(path, again)
        self.assertEqual(path['poses'][0]['x'], path['poses'][-1]['x'])
        self.assertEqual(path['poses'][0]['y'], path['poses'][-1]['y'])
        p = path['planning']
        self.assertGreaterEqual(p['minimum_route_clearance_m'], .2)
        self.assertGreater(p['perimeter_length_m'], 10)
        self.assertGreater(p['hole_area_m2'], 1)
        scene = InteriorGSScene('data', seed=1, clearance=.2,
            spawn_xy=(path['poses'][0]['x'], path['poses'][0]['y']))
        scene.validate_path(path)
        for r,c in p['route_pixel_rc'][::30]:
            self.assertEqual(scene.world_to_pixel(*scene.pixel_to_world(r,c)), (r,c))
    def test_no_hole(self):
        image = np.full((60,60),255,dtype=np.uint8)
        with self.assertRaisesRegex(ValueError,'No closed perimeter'):
            plan_loop(image,.05,[0,0],.2)
    def test_no_free_space(self):
        with self.assertRaisesRegex(ValueError,'No free pixel'):
            plan_loop(np.zeros((20,20),dtype=np.uint8),.05,[0,0],.2)

if __name__ == '__main__': unittest.main()
