"""Unit tests for dropping the robot from the camera cloud (core/self_filter.py)."""

import numpy as np

from skuba_navigation.core.self_filter import inside, not_the_robot, widen

# A 0.3 m robot holding a bag 0.55 m out in front: the footprint arm_footprint publishes.
FOOTPRINT = np.array([[-0.3, -0.3], [0.6, -0.3], [0.6, 0.3], [-0.3, 0.3]])


def test_inside_a_square():
    hit = inside(FOOTPRINT, [[0.0, 0.0], [0.55, 0.2], [0.7, 0.0], [0.0, -0.4]])
    assert hit.tolist() == [True, True, False, False]


def test_widen_moves_every_vertex_out():
    wide = widen(FOOTPRINT, 0.1)
    assert inside(wide, [[0.65, 0.0], [0.0, 0.35]]).all()
    assert not inside(wide, [[0.9, 0.0]]).any()


def test_the_carried_bag_goes_and_a_shoe_ahead_stays():
    points = np.array(
        [
            [0.55, 0.05, 0.6],  # the bag in the hand
            [0.62, -0.1, 0.4],  # its edge, just past the footprint (inside the margin)
            [1.4, 0.0, 0.08],  # a shoe on the floor ahead
            [0.0, 0.0, 1.2],  # the camera's own mount
        ]
    )
    keep = not_the_robot(points, FOOTPRINT, margin=0.12)
    assert keep.tolist() == [False, False, True, False]


def test_without_a_footprint_nothing_is_dropped():
    assert not_the_robot(np.zeros((3, 3)), np.zeros((0, 2)), 0.1).all()
