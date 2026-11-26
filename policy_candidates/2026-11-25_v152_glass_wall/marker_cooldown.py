"""Reduce ordinary exploration gain around a registered marker, not approach priority."""
import math


def exploration_gain(point, targets, radius_m=2., floor=.18):
    if not targets:
        return 1.
    nearest = min(math.dist(point, target) for target in targets)
    if nearest >= radius_m:
        return 1.
    # Keep a narrow continuous edge to avoid goal ranking flicker at two metres.
    if nearest <= radius_m - .2:
        return floor
    return floor + (1. - floor) * (nearest - (radius_m - .2)) / .2
