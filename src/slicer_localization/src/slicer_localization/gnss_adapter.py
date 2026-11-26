"""Associate raw GNSS position, fix status, and horizontal uncertainty."""

from dataclasses import dataclass
import math
import re
from typing import Mapping, Optional, Tuple

from .geographic_transform import GeoPoint, finite


UNKNOWN, SINGLE, FLOAT, FIXED = range(4)
Matrix2 = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Fix:
    # The current receiver stamps NavSatFix when the host receives its NMEA.
    # The receive stamp here is from this consumer, after ROS transport.
    stamp_ns: int
    received_ns: int
    point: Optional[GeoPoint]
    status: int
    frame: str
    covariance: Tuple[float, ...] = ()
    covariance_type: int = 0


@dataclass(frozen=True)
class FixQuality:
    stamp_ns: int
    received_ns: int
    values: Mapping[str, str]


@dataclass(frozen=True)
class FixObservation:
    fix: Fix
    fix_type: int
    valid: bool
    usable: bool
    covariance_en: Matrix2
    covariance_source: str
    receiver_session: str
    reason: str
    quality_matched: bool

    @property
    def covariance_measured(self):
        return self.covariance_source in ("receiver_gst", "navsat_covariance")

    def fresh(self, now_ns, max_age_ns):
        return (0 < self.fix.stamp_ns <= self.fix.received_ns <= now_ns
                and now_ns - self.fix.stamp_ns <= max_age_ns)


def _utc_epoch(value):
    """Canonicalize the same receiver's undated NMEA UTC field exactly."""
    if not isinstance(value, str) or not re.fullmatch(r"\d{6}(?:\.\d+)?", value):
        return ""
    if int(value[:2]) >= 24 or int(value[2:4]) >= 60 or int(value[4:6]) >= 60:
        return ""
    return value.rstrip("0").rstrip(".") if "." in value else value


def _positive(value):
    try:
        parsed = float(value)
    except (ValueError, TypeError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) and parsed > 0.0 else None


def _covariance(fix, values, fallback_variance):
    gga, gst = map(_utc_epoch, (values.get("gga_utc_time"), values.get("receiver_gst_utc_time")))
    declared_match = values.get("receiver_gst_gga_match")
    if values.get("session") and gga and gga == gst and declared_match != "false":
        # The paired diagnostic was emitted by the same receiver transport
        # session. GST longitude and latitude std deviations map to East/North.
        east = _positive(values.get("receiver_gst_lon_std_m"))
        north = _positive(values.get("receiver_gst_lat_std_m"))
        if east is not None and north is not None:
            return (east * east, 0.0, 0.0, north * north), "receiver_gst"
    if fix.covariance_type in (2, 3) and len(fix.covariance) == 9:
        a, b, c, d = (fix.covariance[index] for index in (0, 1, 3, 4))
        if all(finite(value) for value in (a, b, c, d)):
            if a > 0 and d > 0 and abs(b - c) <= 1e-8 and a * d > b * c:
                cross = 0.5 * (b + c)
                return (a, cross, cross, d), "navsat_covariance"
    return (fallback_variance, 0.0, 0.0, fallback_variance), "conservative_assumption"


class FixAdapter:
    def __init__(self, frame="gnss_link", max_age_ns=2_500_000_000,
                 association_wait_ns=250_000_000, assumed_std_m=0.5):
        if not frame or not 0 < association_wait_ns < max_age_ns:
            raise ValueError("invalid GNSS association parameters")
        if not finite(assumed_std_m) or assumed_std_m <= 0.0:
            raise ValueError("GNSS fallback uncertainty must be positive")
        self.frame = frame
        self.max_age_ns = max_age_ns
        self.association_wait_ns = association_wait_ns
        self.fallback_variance = assumed_std_m ** 2
        self.reset()

    def reset(self):
        self.fixes = {}
        self.qualities = {}
        self.last_stamp_ns = 0

    def _add(self, records, message):
        if (self.last_stamp_ns < message.stamp_ns <= message.received_ns
                and message.stamp_ns not in records):
            records[message.stamp_ns] = message
            if len(records) > 256:
                del records[min(records)]

    def add_fix(self, message):
        self._add(self.fixes, message)

    def add_quality(self, message):
        self._add(self.qualities, message)

    def _observation(self, fix, quality, now_ns):
        values = quality.values if quality is not None else {}
        fix_type = {"1": SINGLE, "2": SINGLE, "4": FIXED, "5": FLOAT}.get(
            str(values.get("gga_quality")), UNKNOWN)
        covariance, source = _covariance(fix, values, self.fallback_variance)
        session = values.get("session", "")
        reason = ""
        if quality is None:
            reason = "No receiver status matched this fix's exact source stamp"
        elif now_ns - fix.stamp_ns > self.max_age_ns:
            reason = "GNSS source stamp is stale"
        elif fix.frame != self.frame:
            reason = "Expected GNSS antenna frame"
        elif fix.status < 0 or fix.point is None or str(values.get("fix_valid")).lower() != "true":
            reason = "Receiver reports an invalid GNSS fix"
        if reason:
            return FixObservation(fix, fix_type, False, False, covariance, source, session, reason,
                                  quality is not None)
        if fix_type != FIXED or str(values.get("fresh_rtk_usable")).lower() != "true":
            reason = "Receiver reports no fresh RTK Fixed solution"
        elif not session or not _utc_epoch(values.get("gga_utc_time")):
            reason = "GNSS measurement UTC or receiver session is missing"
        elif fix.point.altitude_ellipsoid_m is None:
            reason = "GNSS ellipsoid height is missing"
        elif not all(finite(value) for value in covariance):
            reason = "Receiver horizontal uncertainty is invalid"
        return FixObservation(fix, fix_type, True, not reason, covariance, source, session, reason, True)

    def poll(self, now_ns):
        observations = []
        for stamp in sorted(self.fixes):
            fix = self.fixes[stamp]
            quality = self.qualities.get(stamp)
            if fix.received_ns > now_ns or (quality and quality.received_ns > now_ns):
                break
            if quality is None and now_ns - fix.received_ns < self.association_wait_ns:
                break
            observations.append(self._observation(fix, quality, now_ns))
            self.last_stamp_ns = stamp
            del self.fixes[stamp]
        self.qualities = {stamp: quality for stamp, quality in self.qualities.items()
                          if stamp > self.last_stamp_ns and now_ns - stamp <= self.max_age_ns}
        return observations
