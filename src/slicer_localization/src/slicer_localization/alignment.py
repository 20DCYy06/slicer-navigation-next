"""Fit the existing planar rigid alignment to paired antenna trajectories."""

from bisect import bisect_left
from dataclasses import dataclass
import math
from statistics import median

from .geographic_transform import MapFromEnu, XY, finite, rotate_xy, wrap_yaw


@dataclass(frozen=True)
class PlanarPose:
    stamp_ns: int
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if type(self.stamp_ns) is not int or self.stamp_ns <= 0:
            raise ValueError("pose requires a positive source stamp in nanoseconds")
        if not all(finite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("pose x, y and yaw must be finite")

    @property
    def xy(self):
        return self.x, self.y

    def antenna_xy(self, antenna_offset):
        dx, dy = rotate_xy(antenna_offset, self.yaw)
        return self.x + dx, self.y + dy


@dataclass(frozen=True)
class AlignmentPair:
    map_pose: PlanarPose
    antenna_en: XY

    def __post_init__(self):
        if len(self.antenna_en) != 2 or not all(finite(value) for value in self.antenna_en):
            raise ValueError("alignment requires a finite horizontal antenna position")


def interpolate_pose(samples, stamp_ns, max_gap_ns):
    """Interpolate position and shortest-arc yaw in the source clock domain."""
    index = bisect_left([sample.stamp_ns for sample in samples], stamp_ns)
    if index < len(samples) and samples[index].stamp_ns == stamp_ns:
        return samples[index]
    if index == 0 or index == len(samples):
        raise ValueError("pose timestamp falls outside observed coverage")
    left, right = samples[index - 1], samples[index]
    interval = right.stamp_ns - left.stamp_ns
    if not 0 < interval <= max_gap_ns:
        raise ValueError("pose timestamps exceed allowed interpolation gap")
    fraction = (stamp_ns - left.stamp_ns) / interval
    return PlanarPose(stamp_ns, left.x + fraction * (right.x - left.x),
                      left.y + fraction * (right.y - left.y),
                      wrap_yaw(left.yaw + fraction * wrap_yaw(right.yaw - left.yaw)))


@dataclass(frozen=True)
class AlignmentLimits:
    min_baseline_m: float = 1.2
    spacing_m: float = 0.20
    min_samples: int = 6
    max_gap_ns: int = 1_500_000_000
    max_residual_m: float = 0.15
    max_heading_std_rad: float = 0.11
    max_heading_delta_rad: float = 0.10

    def __post_init__(self):
        thresholds = (self.min_baseline_m, self.spacing_m, self.max_residual_m,
                      self.max_heading_std_rad, self.max_heading_delta_rad)
        if (not all(finite(value) and value > 0 for value in thresholds)
                or type(self.min_samples) is not int or self.min_samples < 5
                or type(self.max_gap_ns) is not int or self.max_gap_ns <= 0):
            raise ValueError("invalid geographic alignment limits")


def _baseline(pairs):
    first = pairs[0]
    return min(max(math.dist(first.map_pose.xy, pair.map_pose.xy) for pair in pairs),
               max(math.dist(first.antenna_en, pair.antenna_en) for pair in pairs))


def _rigid_fit(pairs, antenna_offset, minimum_baseline):
    baseline = _baseline(pairs)
    if baseline < minimum_baseline:
        raise ValueError("insufficient spatial baseline")
    sources = [pair.antenna_en for pair in pairs]
    destinations = [pair.map_pose.antenna_xy(antenna_offset) for pair in pairs]
    source_mean = tuple(math.fsum(axis) / len(pairs) for axis in zip(*sources))
    destination_mean = tuple(math.fsum(axis) / len(pairs) for axis in zip(*destinations))
    dot = cross = 0.0
    for (east, north), (x, y) in zip(sources, destinations):
        east, north = east - source_mean[0], north - source_mean[1]
        x, y = x - destination_mean[0], y - destination_mean[1]
        dot += east * x + north * y
        cross += east * y - north * x
    if not math.isfinite(math.hypot(dot, cross)) or math.hypot(dot, cross) < 1e-12:
        raise ValueError("paired trajectories have no identifiable common heading")
    yaw = math.atan2(cross, dot)
    mean_x, mean_y = rotate_xy(source_mean, yaw)
    translation = destination_mean[0] - mean_x, destination_mean[1] - mean_y
    provisional = MapFromEnu(yaw, translation, 0.0, 0.0)
    residuals = [math.dist(provisional.project_xy(source), destination)
                 for source, destination in zip(sources, destinations)]
    rmse = math.sqrt(math.fsum(residual ** 2 for residual in residuals) / len(pairs))
    heading_std = max(math.radians(5.0), math.atan2(rmse, baseline))
    return MapFromEnu(yaw, translation, rmse, heading_std), residuals


def fit_trajectory(pairs, antenna_offset, limits=AlignmentLimits()):
    """Use the latest continuous set of spatially distinct, same-epoch pairs."""
    selected = []
    previous = None
    for pair in pairs:
        current = pair.map_pose.stamp_ns
        if previous is not None:
            if current <= previous:
                raise ValueError("alignment timestamps must strictly increase")
            if current - previous > limits.max_gap_ns:
                selected.clear()
        previous = current
        if all(math.dist(pair.map_pose.xy, other.map_pose.xy) >= limits.spacing_m
               for other in selected):
            selected.append(pair)
    if len(selected) < limits.min_samples:
        raise ValueError("alignment needs more distinct positions")
    fit, residuals = _rigid_fit(selected, antenna_offset, limits.min_baseline_m)
    cutoff = max(2.0 * limits.max_residual_m, 3.0 * median(residuals))
    inliers = [pair for pair, residual in zip(selected, residuals) if residual <= cutoff]
    rejected = len(selected) - len(inliers)
    if rejected > max(1, int(0.15 * len(selected))) or len(inliers) < limits.min_samples:
        raise ValueError("too many alignment outliers")
    if rejected:
        fit, _ = _rigid_fit(inliers, antenna_offset, limits.min_baseline_m)
    if fit.rmse_m > limits.max_residual_m:
        raise ValueError("alignment residual exceeds its limit")
    if fit.heading_std_rad > limits.max_heading_std_rad:
        raise ValueError("alignment heading uncertainty exceeds its limit")
    group_size = (2 * len(inliers) + 2) // 3
    early, _ = _rigid_fit(inliers[:group_size], antenna_offset, 0.4 * limits.min_baseline_m)
    late, _ = _rigid_fit(inliers[-group_size:], antenna_offset, 0.4 * limits.min_baseline_m)
    for subfit in (early, late):
        if subfit.rmse_m > limits.max_residual_m:
            raise ValueError("alignment subtrack residual exceeds its limit")
    heading_delta = max(abs(wrap_yaw(first - second)) for first, second in
                        ((early.yaw, late.yaw), (early.yaw, fit.yaw), (late.yaw, fit.yaw)))
    if heading_delta > limits.max_heading_delta_rad:
        raise ValueError("alignment headings disagree across subtracks")
    return fit
