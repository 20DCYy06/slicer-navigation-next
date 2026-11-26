"""Select a bounded preparation path wholly inside observed clear costmap cells."""
import math

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
    radius = max(math.hypot(*point) for point in footprint)
    resolution = grid.info.resolution
    if (not math.isfinite(resolution) or not .01 <= resolution <= .50
            or len(grid.data) != grid.info.width * grid.info.height):
        return None
    start = pose.position
    yaw = yaw_of(pose.orientation)
    clearance = radius + math.sqrt(2) * resolution
    cells = math.ceil(clearance / resolution)
    offsets = [(i * resolution, j * resolution) for i in range(-cells, cells + 1)
               for j in range(-cells, cells + 1) if math.hypot(i, j) * resolution <= clearance]

    def clear(x, y):
        for ox, oy in offsets:
            index = grid_cell(grid, x + ox, y + oy)
            if index is None or not 0 <= grid.data[index] < 85:
                return False
        return True

    steps = math.ceil(distance / (resolution / 2))
    for turn in (0, 1, -1, 2, -2, 3, -3, 4):
        heading = yaw + turn * math.pi / 4
        dx, dy = distance * math.cos(heading), distance * math.sin(heading)
        if all(clear(start.x + dx * step / steps, start.y + dy * step / steps)
               for step in range(steps + 1)):
            return start.x + dx, start.y + dy, heading
    return None
