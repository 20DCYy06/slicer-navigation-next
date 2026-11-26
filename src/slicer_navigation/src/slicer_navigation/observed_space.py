"""Select a bounded preparation path wholly inside observed clear costmap cells."""
import math

from slicer_base.motion_guard import swept_clear

from slicer_navigation.arrival_check import yaw_of


def grid_cell(grid, x, y):
    origin = grid.info.origin.position
    resolution = grid.info.resolution
    if not math.isfinite(resolution) or resolution <= 0:
        return None
    mx = math.floor((x - origin.x) / resolution)
    my = math.floor((y - origin.y) / resolution)
    if not (0 <= mx < grid.info.width and 0 <= my < grid.info.height):
        return None
    return my * grid.info.width + mx


def preparation_candidate(grid, pose, footprint, distance):
    info = grid.info
    if (not .001 <= info.resolution <= 1
            or not info.width or not info.height or len(grid.data) != info.width * info.height):
        return None
    origin = info.origin.position.x, info.origin.position.y, yaw_of(info.origin.orientation)
    start = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
    if not all(math.isfinite(value) for value in (*origin, *start)):
        return None
    occupancy = dict(origin=origin, resolution=info.resolution, width=info.width,
                     height=info.height, data=grid.data)
    x, y, yaw = start
    for turn in (0, 1, -1, 2, -2, 3, -3, 4):
        angle = turn * math.pi / 4
        if turn and not swept_clear(start, 0.0, math.copysign(1.0, angle),
                                    footprint, occupancy, abs(angle)):
            continue
        heading = yaw + angle
        if swept_clear((x, y, heading), 1.0, 0.0, footprint, occupancy, distance):
            return x + distance * math.cos(heading), y + distance * math.sin(heading), heading
    return None
