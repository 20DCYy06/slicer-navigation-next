"""Session-local WGS84 -> ENU -> map, with a horizontal antenna lever arm."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
XY = Tuple[float, float]


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def wrap_yaw(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def rotate_xy(point, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    x, y = point
    return cosine * x - sine * y, sine * x + cosine * y


@dataclass(frozen=True)
class GeoPoint:
    latitude: float
    longitude: float
    altitude_ellipsoid_m: Optional[float] = None

    def __post_init__(self):
        if not finite(self.latitude) or not -90.0 <= self.latitude <= 90.0:
            raise ValueError("WGS84 latitude outside [-90, 90]")
        if not finite(self.longitude) or not -180.0 <= self.longitude <= 180.0:
            raise ValueError("WGS84 longitude outside [-180, 180]")
        if self.altitude_ellipsoid_m is not None and not finite(self.altitude_ellipsoid_m):
            raise ValueError("WGS84 ellipsoid height must be finite")


def ecef(point, altitude):
    lat, lon = math.radians(point.latitude), math.radians(point.longitude)
    slat, clat = math.sin(lat), math.cos(lat)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * slat * slat)
    return ((normal + altitude) * clat * math.cos(lon),
            (normal + altitude) * clat * math.sin(lon),
            (normal * (1.0 - WGS84_E2) + altitude) * slat)


class EnuFrame:
    def __init__(self, origin):
        if origin.altitude_ellipsoid_m is None:
            raise ValueError("ENU origin requires a WGS84 ellipsoid height")
        self.origin = origin
        self._ecef = ecef(origin, origin.altitude_ellipsoid_m)
        lat, lon = math.radians(origin.latitude), math.radians(origin.longitude)
        slat, clat = math.sin(lat), math.cos(lat)
        slon, clon = math.sin(lon), math.cos(lon)
        self._axes = ((-slon, clon, 0.0),
                      (-slat * clon, -slat * slon, clat),
                      (clat * clon, clat * slon, slat))

    def project(self, point):
        """Goals without height use the origin ellipsoid height for plane projection."""
        altitude = point.altitude_ellipsoid_m
        target = ecef(point, self.origin.altitude_ellipsoid_m if altitude is None else altitude)
        delta = tuple(value - origin for value, origin in zip(target, self._ecef))
        return tuple(sum(axis_value * value for axis_value, value in zip(axis, delta))
                     for axis in self._axes)

    def horizontal_geodetic(self, enu):
        """Recover latitude and longitude from a position near the WGS84 surface."""
        x, y, z = (origin + sum(component * coefficient for component, coefficient in zip(enu, axis))
                   for origin, axis in zip(self._ecef, zip(*self._axes)))
        horizontal = math.hypot(x, y)
        minor = WGS84_A * (1.0 - WGS84_F)
        second_eccentricity_sq = WGS84_E2 / (1.0 - WGS84_E2)
        auxiliary = math.atan2(z * WGS84_A, horizontal * minor)
        latitude = math.atan2(z + second_eccentricity_sq * minor * math.sin(auxiliary) ** 3,
                              horizontal - WGS84_E2 * WGS84_A * math.cos(auxiliary) ** 3)
        return GeoPoint(math.degrees(latitude), math.degrees(math.atan2(y, x)))


@dataclass(frozen=True)
class MapFromEnu:
    yaw: float
    translation: XY
    rmse_m: float
    heading_std_rad: float

    def project_xy(self, east_north):
        east, north = rotate_xy(east_north, self.yaw)
        return east + self.translation[0], north + self.translation[1]

    def base_xy(self, antenna_en, base_yaw_map, antenna_offset):
        antenna_x, antenna_y = self.project_xy(antenna_en)
        lever_x, lever_y = rotate_xy(antenna_offset, base_yaw_map)
        return antenna_x - lever_x, antenna_y - lever_y
