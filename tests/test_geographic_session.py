import math
import pathlib
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]
                       / "src/slicer_localization/src"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]
                       / "src/slicer_gnss/src"))

from slicer_gnss.core import GnssModel
from slicer_localization.alignment import AlignmentPair, PlanarPose, fit_trajectory, interpolate_pose
from slicer_localization.geographic_session import GeographicSession, GeographicUnavailable
from slicer_localization.geographic_transform import EnuFrame, GeoPoint, rotate_xy
from slicer_localization.gnss_adapter import Fix, FixAdapter, FixQuality


SECOND = 1_000_000_000
ORIGIN = GeoPoint(0.0, 0.0, 30.0)
LEVER = (0.065, 0.0)
YAW = 0.7
TRANSLATION = (3.0, -4.0)


def quality(stamp, *, fix_quality=4, gst_utc="010001.2", session="rx-1"):
    return FixQuality(stamp, stamp + 10, {
        "gga_quality": str(fix_quality), "gga_utc_time": "010001.20",
        "session": session, "fix_valid": "true", "fresh_rtk_usable": "true",
        "receiver_gst_utc_time": gst_utc, "receiver_gst_lon_std_m": "0.040",
        "receiver_gst_lat_std_m": "0.060",
    })


def fix(stamp, point=ORIGIN):
    return Fix(stamp, stamp + 8, point, 2, "gnss_link")


def observation(stamp, point=ORIGIN, gst_utc="010001.2"):
    adapter = FixAdapter()
    adapter.add_fix(fix(stamp, point))
    adapter.add_quality(quality(stamp, gst_utc=gst_utc))
    return adapter.poll(stamp + 20)[0]


def gst(epoch, longitude_std="0.040"):
    content = f"GNGST,{epoch},0.08,0.09,0.05,30,0.060,{longitude_std},0.150"
    checksum = 0
    for character in content:
        checksum ^= ord(character)
    return f"${content}*{checksum:02X}"


def track(arc=False, stationary=False, gap=False):
    pairs = []
    for index in range(10):
        distance = index * 0.24
        x = 0.0 if stationary else distance
        y = 0.0 if stationary or not arc else 0.2 * distance ** 2
        yaw = 0.4 * distance if arc or stationary else 0.0
        stamp = SECOND + index * (SECOND // 4)
        if gap and index >= 5:
            stamp += 2 * SECOND
        lever_x, lever_y = rotate_xy(LEVER, yaw)
        east, north = rotate_xy((x + lever_x - TRANSLATION[0],
                                 y + lever_y - TRANSLATION[1]), -YAW)
        pairs.append(AlignmentPair(PlanarPose(stamp, x, y, yaw), (east, north)))
    return pairs


class GeographicSessionTest(unittest.TestCase):
    def test_wgs84_antimeridian_and_optional_ellipsoid_height(self):
        enu = EnuFrame(GeoPoint(0.0, 179.99999, 40.0))
        self.assertAlmostEqual(enu.project(GeoPoint(0.0, -180.0))[0], 1.1132, places=3)
        east, north, up = enu.project(GeoPoint(0.0, 179.99999, 45.0))
        self.assertAlmostEqual(east, 0.0, places=8)
        self.assertAlmostEqual(north, 0.0, places=8)
        self.assertAlmostEqual(up, 5.0, places=7)
        self.assertAlmostEqual(enu.project(GeoPoint(0.0, 179.99999))[2], 0.0, places=8)
        with self.assertRaises(ValueError):
            enu.project(GeoPoint(91.0, 0.0))

    def test_alignment_uses_same_time_rotated_antenna_for_line_and_arc(self):
        for arc in (False, True):
            with self.subTest(arc=arc):
                fit = fit_trajectory(track(arc=arc), LEVER)
                self.assertAlmostEqual(fit.yaw, YAW)
                self.assertAlmostEqual(fit.translation[0], TRANSLATION[0])
                self.assertAlmostEqual(fit.translation[1], TRANSLATION[1])
                self.assertAlmostEqual(fit.rmse_m, 0.0)

    def test_static_turn_and_discontinuous_track_do_not_produce_heading(self):
        for pairs in (track(stationary=True), track(gap=True)):
            with self.assertRaisesRegex(ValueError, "distinct positions|baseline"):
                fit_trajectory(pairs, LEVER)

    def test_pose_interpolation_rejects_extrapolation_and_wraps_yaw(self):
        first = PlanarPose(SECOND, 0.0, 0.0, math.pi - 0.1)
        last = PlanarPose(SECOND + 200_000_000, 1.0, 2.0, -math.pi + 0.1)
        sample = interpolate_pose([first, last], SECOND + 100_000_000, 250_000_000)
        self.assertEqual((sample.x, sample.y), (0.5, 1.0))
        self.assertAlmostEqual(abs(sample.yaw), math.pi)
        for stamp, gap in ((SECOND - 1, SECOND), (SECOND + 1, 100_000_000)):
            with self.assertRaises(ValueError):
                interpolate_pose([first, last], stamp, gap)

    def test_fix_quality_is_epoch_exact_and_keeps_receipt_stamps(self):
        adapter = FixAdapter()
        adapter.add_fix(fix(SECOND))
        adapter.add_quality(quality(SECOND + 1))
        self.assertEqual(adapter.poll(SECOND + 20), [])
        adapter.add_quality(quality(SECOND))
        measured = adapter.poll(SECOND + 20)[0]
        self.assertEqual(measured.fix_type, 3)
        self.assertTrue(measured.usable)
        self.assertEqual(measured.fix.stamp_ns, SECOND)
        self.assertEqual(measured.fix.received_ns, SECOND + 8)
        self.assertEqual(measured.covariance_en, (0.04 ** 2, 0.0, 0.0, 0.06 ** 2))
        self.assertTrue(measured.covariance_measured)
        adapter.add_fix(fix(SECOND))
        self.assertEqual(adapter.poll(SECOND + 20), [])

    def test_bad_fix_invalidates_and_covariance_fallback_is_explicit(self):
        fallback = observation(SECOND, gst_utc="010000.90")
        self.assertFalse(fallback.covariance_measured)
        self.assertEqual(fallback.covariance_en, (0.25, 0.0, 0.0, 0.25))
        adapter = FixAdapter()
        adapter.add_fix(fix(SECOND))
        adapter.add_quality(quality(SECOND, fix_quality=5))
        floating = adapter.poll(SECOND + 20)[0]
        self.assertTrue(floating.valid)
        self.assertFalse(floating.usable)
        self.assertEqual(floating.fix_type, 2)
        self.assertFalse(observation(SECOND).fresh(4 * SECOND, 2_500_000_000))

    def test_receiver_associates_gst_on_both_sides_of_gga_and_refuses_stale_session(self):
        model = GnssModel()
        model.ingest({"kind": "transport", "connected": True}, 10.0, 10.0)
        for sequence, (epoch, receipt) in enumerate((("010001.200", 9.96), ("010001.3", 10.16))):
            model.ingest({"v": 1, "kind": "nmea", "session": "rx", "seq": sequence,
                          "captured_at": receipt, "nmea": gst(epoch)}, receipt, receipt)
        for epoch, gga_stamp in (("010001.2", 10.0), ("010001.300", 10.1)):
            selected = model.gst_at(epoch, "rx", gga_stamp, 0.15)
            self.assertEqual(selected["receiver_error_fields"]["lon_std_dev"], 0.04)
        self.assertIsNone(model.gst_at("010001.4", "rx", 10.0, 0.15))
        self.assertIsNone(model.gst_at("010001.2", "rx", 10.2, 0.15))
        self.assertIsNone(model.gst_at("010001.2", "other-rx", 10.0, 0.15))

    def test_stopped_commit_versions_goals_and_requires_new_fix_for_arrival(self):
        session = GeographicSession(LEVER, ORIGIN)
        with self.assertRaises(GeographicUnavailable):
            session.project(0.0, 0.0)
        session.begin_preparation()
        pairs = track(arc=True)
        ellipsoid_radius = 6378137.0 + ORIGIN.altitude_ellipsoid_m
        north_radius = (6378137.0 * (1 - 0.0066943799901413165)
                        + ORIGIN.altitude_ellipsoid_m)
        for pair in pairs:
            sample = pair.map_pose
            session.add_map_pose(sample)
            session.add_lio_pose(PlanarPose(sample.stamp_ns, sample.x, sample.y, sample.yaw))
            point = GeoPoint(math.degrees(pair.antenna_en[1] / north_radius),
                             math.degrees(pair.antenna_en[0] / ellipsoid_radius), 30.0)
            session.observe(observation(sample.stamp_ns, point), sample.stamp_ns + 20)
        last_stamp = pairs[-1].map_pose.stamp_ns
        self.assertTrue(session.candidate_ready(last_stamp + 20))
        self.assertFalse(session.commit(False, last_stamp + 20))
        self.assertTrue(session.commit(True, last_stamp + 20))
        self.assertEqual(session.version, 1)
        self.assertFalse(session.commit(True, last_stamp + 20))
        projected = session.project(0.0, 0.0, yaw_enu_rad=0.1)
        self.assertAlmostEqual(projected.yaw, 0.8)
        self.assertAlmostEqual(projected.x, 3.0, places=5)
        self.assertAlmostEqual(projected.y, -4.0, places=5)
        self.assertFalse(session.verify(projected, last_stamp, last_stamp + 20, 0.2).valid)

        stamp = last_stamp + 100_000_000
        sample = pairs[-1].map_pose
        session.add_map_pose(PlanarPose(stamp, sample.x, sample.y, sample.yaw))
        session.add_lio_pose(PlanarPose(stamp, sample.x, sample.y, sample.yaw))
        # WGS84 goal is the base origin. Put the antenna one lever arm from it.
        offset = rotate_xy(LEVER, pairs[-1].map_pose.yaw - YAW)
        antenna = GeoPoint(math.degrees(offset[1] / north_radius),
                           math.degrees(offset[0] / ellipsoid_radius), 30.0)
        session.observe(observation(stamp, antenna), stamp + 20)
        result = session.verify(projected, last_stamp, stamp + 20, 0.2)
        self.assertTrue(result.valid)
        self.assertTrue(result.arrived)
        self.assertEqual(result.stamp_ns, stamp)
        self.assertAlmostEqual(result.error_m, 0.0, places=5)
        # At 20 Hz, GGA usually advances again before the next 1 Hz GST.
        fallback = stamp + 5_000_000
        session.add_map_pose(PlanarPose(fallback, sample.x, sample.y, sample.yaw))
        session.add_lio_pose(PlanarPose(fallback, sample.x, sample.y, sample.yaw))
        session.observe(observation(fallback, antenna, gst_utc="010000.90"), fallback + 20)
        result = session.verify(projected, last_stamp, fallback + 20, 0.2)
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(result.stamp_ns, stamp)
        self.assertFalse(session.verify(projected, stamp, fallback + 20, 0.2).valid)
        # A newer usable fix may be waiting for its future LIO bracket.
        pending = stamp + 10_000_000
        session.observe(observation(pending, antenna), pending + 20)
        self.assertTrue(session.verify(projected, last_stamp, pending + 20, 0.2).valid)
        broad = pending + 10_000_000
        session.add_map_pose(PlanarPose(broad, sample.x, sample.y, sample.yaw))
        session.add_lio_pose(PlanarPose(broad, sample.x, sample.y, sample.yaw))
        uncertain = replace(observation(broad, antenna), covariance_en=(0.25, 0.0, 0.0, 0.25))
        session.observe(uncertain, broad + 20)
        self.assertIn("uncertainty", session.verify(projected, last_stamp, broad + 20, 0.2).reason)
        fresh = broad + 5_000_000
        session.add_map_pose(PlanarPose(fresh, sample.x, sample.y, sample.yaw))
        session.add_lio_pose(PlanarPose(fresh, sample.x, sample.y, sample.yaw))
        session.observe(observation(fresh, antenna), fresh + 20)
        self.assertTrue(session.verify(projected, last_stamp, fresh + 20, 0.2).valid)
        late = fresh + 3 * SECOND
        session.add_map_pose(PlanarPose(late, sample.x, sample.y, sample.yaw))
        session.add_lio_pose(PlanarPose(late, sample.x, sample.y, sample.yaw))
        session.observe(observation(late, antenna, gst_utc="010000.90"), late + 20)
        self.assertFalse(session.verify(projected, last_stamp, late + 20, 0.2).valid)
        session.add_map_pose(PlanarPose(stamp - 1, 999.0, 999.0, 0.0))
        self.assertFalse(session.verify(projected, last_stamp, stamp + 20, 0.2).valid)

    def test_aborted_preparation_has_no_commit(self):
        session = GeographicSession(LEVER, ORIGIN)
        session.begin_preparation()
        session.cancel_preparation()
        self.assertFalse(session.commit(True, 4 * SECOND))

    def test_slow_preparation_retains_the_baseline_at_receiver_rate(self):
        session = GeographicSession(LEVER, ORIGIN)
        session.begin_preparation(SECOND)
        radius = 6378137.0 + ORIGIN.altitude_ellipsoid_m
        for index in range(401):
            stamp = SECOND + index * 50_000_000
            x = index * 0.004375
            pose = PlanarPose(stamp, x, 0.0, 0.0)
            session.add_map_pose(pose)
            session.add_lio_pose(pose)
            point = GeoPoint(0.0, math.degrees((x + LEVER[0]) / radius), ORIGIN.altitude_ellipsoid_m)
            session.observe(observation(stamp, point), stamp + 20)
        self.assertTrue(session.candidate_ready(stamp + 20), session.reason)
        self.assertGreater(len(session.pairs), 256)

    def test_preparation_history_is_bounded_by_source_time(self):
        session = GeographicSession(LEVER, ORIGIN, preparation_timeout_ns=SECOND)
        session.begin_preparation(SECOND)
        stamps = [SECOND, SECOND + 500_000_000, 2 * SECOND + 200_000_000]
        for stamp in stamps:
            pose = PlanarPose(stamp, 0.0, 0.0, 0.0)
            session.add_map_pose(pose)
            session.add_lio_pose(pose)
            session.observe(observation(stamp), stamp + 20)
        self.assertEqual([pair.map_pose.stamp_ns for pair in session.pairs], stamps[1:])


if __name__ == "__main__":
    unittest.main()
