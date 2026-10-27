"""
Drop the robot itself from the head camera's cloud before the costmaps mark it.

With the head turned (following a person) the camera sees the arm and what the
hand carries; marked, they sit right on the footprint's edge and the controller
stops on "collision ahead". The costmap's published footprint already holds
them (skuba_manipulation's arm_footprint grows it round the arm and what it
holds), so every point whose ground projection is inside that footprint,
widened by a margin, is the robot. Pure numpy -- no rclpy import.
"""

import numpy as np


def widen(polygon, margin):
    """Return polygon (k x 2) pushed out by margin from its centroid, each vertex."""
    polygon = np.asarray(polygon, dtype=float)
    centre = polygon.mean(axis=0)
    out = polygon - centre
    r = np.linalg.norm(out, axis=1, keepdims=True)
    return centre + out * (r + margin) / np.maximum(r, 1e-9)


def inside(polygon, xy):
    """Return a bool per row of xy (n x 2): inside polygon (k x 2), even-odd rule."""
    polygon = np.asarray(polygon, dtype=float)
    xy = np.asarray(xy, dtype=float)
    x, y = xy[:, 0], xy[:, 1]
    hit = np.zeros(len(xy), dtype=bool)
    for (x0, y0), (x1, y1) in zip(polygon, np.roll(polygon, -1, axis=0)):
        crosses = (y0 > y) != (y1 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_at = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
        hit ^= crosses & (x < x_at)
    return hit


def not_the_robot(points_base, footprint_base, margin):
    """
    Return a bool per point (n x 3, base frame): True where it is not the robot.

    footprint_base: the published footprint (k x 2) in the same frame.
    """
    points_base = np.asarray(points_base, dtype=float)
    if len(points_base) == 0 or len(footprint_base) < 3:
        return np.ones(len(points_base), dtype=bool)
    return ~inside(widen(footprint_base, margin), points_base[:, :2])
