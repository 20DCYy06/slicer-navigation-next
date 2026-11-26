"""Swept-footprint geometry from ugv_slicer_control/navigation_core.py.

Source is slicer-ugv 34e599365da1f8a79b034d447649d0f43049754f.
"""

import math


class MotionLimits:
    def __init__(self, settings):
        names = ("max_linear_mps", "max_angular_rps", "linear_acceleration_mps2",
                 "angular_acceleration_rps2", "linear_braking_mps2")
        for name in names:
            value = float(settings[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
            setattr(self, name, value)
        vertices = settings["footprint"]
        if len(vertices) < 3 or any(len(point) != 2 for point in vertices):
            raise ValueError("footprint requires at least three planar vertices")
        self.footprint = tuple(tuple(map(float, point)) for point in vertices)
        turns = []
        for i, (x, y) in enumerate(self.footprint):
            qx, qy = self.footprint[(i + 1) % len(self.footprint)]
            rx, ry = self.footprint[(i + 2) % len(self.footprint)]
            turns.append((qx - x) * (ry - qy) - (qy - y) * (rx - qx))
        if (not all(math.isfinite(turn) for turn in turns)
                or not (all(turn > 0 for turn in turns) or all(turn < 0 for turn in turns))):
            raise ValueError("footprint must be a finite convex polygon")

    def clip(self, command):
        linear, angular = command
        return (max(-self.max_linear_mps, min(self.max_linear_mps, linear)),
                max(-self.max_angular_rps, min(self.max_angular_rps, angular)))

    def output(self, command, previous, dt):
        linear, angular = self.clip(command)
        linear_step = self.linear_acceleration_mps2 * dt
        angular_step = self.angular_acceleration_rps2 * dt
        return (max(previous[0] - linear_step, min(previous[0] + linear_step, linear)),
                max(previous[1] - angular_step, min(previous[1] + angular_step, angular)))

    def horizon(self, twist, timeout):
        linear, lateral, angular = twist
        return max(.8, math.hypot(linear, lateral) / self.linear_braking_mps2 + timeout,
                   abs(angular) / self.angular_acceleration_rps2 + timeout)


def intersect(a, b):
    """Convex separating axis test. Contact counts as a collision."""
    for polygon in (a, b):
        for i, p in enumerate(polygon):
            q = polygon[(i + 1) % len(polygon)]
            axis = (-(q[1] - p[1]), q[0] - p[0])
            pa = [x * axis[0] + y * axis[1] for x, y in a]
            pb = [x * axis[0] + y * axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def footprint_clear(pose, footprint, grid, threshold=100, unknown=True, outside_is_blocked=True):
    """The grid uses a planar origin (x, y, yaw) and row-major occupancy data."""
    x, y, yaw = pose
    ox, oy, oa = grid['origin']
    ca, sa = math.cos(oa), math.sin(oa)
    c, s = math.cos(yaw), math.sin(yaw)
    polygon = []
    for px, py in footprint:
        wx, wy = x + c * px - s * py - ox, y + s * px + c * py - oy
        polygon.append(((ca * wx + sa * wy) / grid['resolution'],
                        (-sa * wx + ca * wy) / grid['resolution']))
    x0, x1 = math.floor(min(p[0] for p in polygon)), math.floor(max(p[0] for p in polygon))
    y0, y1 = math.floor(min(p[1] for p in polygon)), math.floor(max(p[1] for p in polygon))
    if outside_is_blocked and (x0 < 0 or y0 < 0 or x1 >= grid['width'] or y1 >= grid['height']):
        return False
    for iy in range(max(y0, 0), min(y1 + 1, grid['height'])):
        for ix in range(max(x0, 0), min(x1 + 1, grid['width'])):
            value = grid['data'][iy * grid['width'] + ix]
            if value >= threshold or (value < 0 and unknown):
                cell = [(ix, iy), (ix + 1, iy), (ix + 1, iy + 1), (ix, iy + 1)]
                if intersect(polygon, cell):
                    return False
    return True


def body_twist_between(start, end, dt):
    """Estimate a constant body-frame planar twist between consecutive LIO poses."""
    angle = end[2] - start[2]
    angle = math.atan2(math.sin(angle), math.cos(angle))
    half = .5 * angle
    mid_yaw = start[2] + half
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(mid_yaw), math.sin(mid_yaw)
    duration = dt * (math.sin(half) / half if abs(half) > 1e-9 else 1.)
    return (c*dx + s*dy) / duration, (c*dy - s*dx) / duration, angle / dt


def swept_clear(pose, v, w, footprint, grid, horizon, threshold=100, unknown=True,
                lateral=0.):
    extent = max(math.hypot(*p) for p in footprint)
    # Sample translation and the rotating footprint edge at half a cell or less.
    speed = math.hypot(v, lateral) + abs(w) * extent
    dt = min(0.1, grid['resolution'] / (2 * speed)) if speed else 0.1
    steps = max(1, math.ceil(horizon / dt))
    x, y, yaw = pose
    for step in range(steps + 1):
        t = horizon * step / steps
        if abs(w) < 1e-9:
            c, s = math.cos(yaw), math.sin(yaw)
            p = (x + t * (v*c - lateral*s), y + t * (v*s + lateral*c), yaw)
        else:
            end_yaw = yaw + w*t
            dsin = math.sin(end_yaw) - math.sin(yaw)
            dcos = math.cos(end_yaw) - math.cos(yaw)
            p = (x + (v*dsin + lateral*dcos) / w,
                 y + (lateral*dsin - v*dcos) / w, end_yaw)
        if not footprint_clear(p, footprint, grid, threshold, unknown):
            return False
    return True


