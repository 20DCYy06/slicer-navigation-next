from pathlib import Path
import math
import sys
import unittest

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

SOURCE = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE / "slicer_navigation/src"))
sys.path.insert(0, str(SOURCE / "slicer_base/src"))
from slicer_navigation.observed_space import preparation_candidate


FOOTPRINT = ((-.145, -.14), (.2, -.14), (.2, .14), (-.145, .14))


def make_grid(occupancy):
    grid = OccupancyGrid()
    grid.header.frame_id = "odom"
    grid.info.resolution = .05
    grid.info.width = grid.info.height = 110
    origin = grid.info.origin
    origin.position.x = origin.position.y = -2.75
    origin.orientation.w = 1.0
    grid.data = [occupancy(-2.75 + (x + .5) * .05, -2.75 + (y + .5) * .05)
                 for y in range(grid.info.height) for x in range(grid.info.width)]
    return grid


class PreparationSpaceTest(unittest.TestCase):
    def setUp(self):
        self.pose = Pose()
        self.pose.orientation.w = 1.0

    def test_known_straight_footprint_with_unknown_cells_alongside(self):
        grid = make_grid(lambda x, y: 0 if -.25 < x < 2.3 and abs(y) < .18 else -1)
        self.assertEqual(preparation_candidate(grid, self.pose, FOOTPRINT, 2.0), (2.0, 0.0, 0.0))

    def test_stopping_reserve_keeps_full_corridor_check(self):
        def corridor(x, y):
            return 0 if -.25 < x < 2.3 and abs(y) < .18 else -1

        self.assertEqual(preparation_candidate(make_grid(corridor), self.pose, FOOTPRINT, 2.0,
                                               stop_reserve=.25), (1.75, 0.0, 0.0))
        blocked = make_grid(lambda x, y: 100 if 2.05 < x < 2.15 and abs(y) < .05 else corridor(x, y))
        self.assertIsNone(preparation_candidate(blocked, self.pose, FOOTPRINT, 2.0, stop_reserve=.25))

    def test_unknown_or_occupied_cell_in_footprint_blocks_corridor(self):
        for occupied in (-1, 100):
            with self.subTest(occupied=occupied):
                def occupancy(x, y):
                    if .75 < x < .85 and abs(y) < .05:
                        return occupied
                    return 0 if -.25 < x < 2.3 and abs(y) < .18 else -1

                self.assertIsNone(preparation_candidate(make_grid(occupancy), self.pose, FOOTPRINT, 2.0))

    def test_sideways_corridor_requires_a_clear_turn_to_enter(self):
        def occupancy(x, y):
            if (abs(x) < .21 and abs(y) < .16) or (abs(x) < .16 and 0 <= y < 2.3):
                return 0
            return -1

        grid = make_grid(occupancy)
        self.pose.orientation.z = self.pose.orientation.w = math.sqrt(.5)
        target = preparation_candidate(grid, self.pose, FOOTPRINT, 2.0)
        self.assertAlmostEqual(target[1], 2.0)
        self.pose.orientation.z, self.pose.orientation.w = 0.0, 1.0
        self.assertIsNone(preparation_candidate(grid, self.pose, FOOTPRINT, 2.0))
