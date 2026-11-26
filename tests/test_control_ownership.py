from pathlib import Path
import struct
import sys

import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/slicer_base/src'))

from slicer_base.driver_core import ServoMailbox, StopObservation, WheelSample
from slicer_base.motion_guard import MotionLimits, swept_clear
from slicer_base.velocity_mux import VelocityMux


def online_mux():
    mux = VelocityMux(timeout=0.25)
    mux.set_online(True, 1.0)
    assert mux.release(1.01)[0]
    return mux


class ControlOwnership(unittest.TestCase):
    def test_hold_and_release_discard_cached_commands_and_reject_previous_epoch(self):
        mux = online_mux()
        old = mux.epoch
        assert mux.authorize('a', True, old, 1.02)[0]
        assert mux.command('nav', (0.2, 0.0), 1.03, 1.03, 2.0, 2.0, 'a')
        assert mux.select(1.04) == ('nav', (0.2, 0.0))
        assert mux.hold(1.05)[0]
        assert mux.release(1.06)[0]
        assert mux.select(1.07) == ('zero', (0.0, 0.0))
        assert not mux.authorize('a', True, old, 1.08)[0]
        assert not mux.command('nav', (0.2, 0.0), 1.08, 1.08, 2.0, 2.0, 'a')


    def test_stale_goal_disable_does_not_revoke_replacement(self):
        mux = online_mux()
        assert mux.authorize('a', True, mux.epoch, 1.02)[0]
        assert mux.authorize('b', True, mux.epoch, 1.03)[0]
        assert not mux.authorize('a', False, mux.epoch, 1.04)[0]
        assert mux.command('nav', (0.2, 0.0), 1.05, 1.05, 4., 4., 'b')
        assert mux.select(1.06) == ('nav', (0.2, 0.0))
        assert mux.select(1.31) == ('zero', (0.0, 0.0))


    def test_manual_timeout_preserves_takeover_and_estop_is_latched(self):
        mux = online_mux()
        assert mux.authorize('a', True, mux.epoch, 1.02)[0]
        mux.takeover(1.03)
        assert mux.command('teleop', (0.2, 0.0), 1.04, 1.04, 3.0, 3.0)
        assert mux.select(1.05) == ('teleop', (0.2, 0.0))
        assert not mux.release(1.06)[0]
        assert mux.select(1.40) == ('zero', (0.0, 0.0))
        assert mux.manual
        mux.emergency(True, 1.41)
        mux.emergency(False, 1.42)
        assert not mux.release(1.43)[0]
        assert mux.estop
        assert mux.select(1.43) == ('zero', (0.0, 0.0))


    def test_delayed_and_replayed_stamped_commands_cannot_refresh_motion(self):
        mux = online_mux()
        assert mux.command('mission', (0.1, 0.0), 1.02, 1.02, 5.0, 5.0)
        assert not mux.command('mission', (0.2, 0.0), 1.10, 1.10, 5.0, 5.08)
        assert not mux.command('mission', (0.2, 0.0), 1.11, 1.11, 4.0, 5.09)
        assert not mux.command('mission', (0.2, 0.0), 1.12, 1.12, 5.20, 5.10)
        assert mux.select(1.28) == ('zero', (0.0, 0.0))


    def test_mission_handoff_drops_navigation_cache_and_authority(self):
        mux = online_mux()
        assert mux.authorize('nav', True, mux.epoch, 1.02)[0]
        assert mux.command('nav', (0.2, 0.0), 1.03, 1.03, 5.01, 5.01, 'nav')
        assert mux.command('mission', (-0.1, 0.0), 1.04, 1.04, 5.02, 5.02)
        assert mux.select(1.05) == ('mission', (-0.1, 0.0))
        assert not mux.goal_id
        assert not mux.authorize('nav', True, mux.epoch, 1.05)[0]
        assert mux.select(1.30) == ('zero', (0.0, 0.0))


    def test_fresh_driver_fault_clears_old_motion_and_tracks_stop_observation(self):
        mux = online_mux()
        assert mux.authorize('a', True, mux.epoch, 1.02)[0]
        previous = mux.epoch
        mux.set_online(False, 1.03)
        mux.set_online(True, 1.04)
        assert mux.select(1.05) == ('zero', (0.0, 0.0))
        assert not mux.authorize('a', True, previous, 1.06)[0]

        observed = StopObservation(0.02, 0.05, 1.0, 0.3)
        for tick in range(5):
            now = 1.0 + tick / 4
            observed.update(now, now, 0.01, 0.04, (0.01, -0.01, 0, 0))
        assert observed.state(2.0) == (True, True)
        assert observed.state(2.31) == (False, False)
        observed.update(2.4, 2.4, 0, 0, (0.03, -0.03, 0, 0))
        assert observed.state(2.4) == (True, False)


    def test_wheel_counter_wrap_and_encoder_signs(self):
        wheel = WheelSample((-1, 1, -1, 1))
        assert wheel.update((0xfffffff0, (-2147483648, 0, 0, 0))) is None
        delta, dt = wheel.update((0x30, (2147483647, 2, -3, 4)))
        assert delta == [1, 2, 3, 4]
        assert dt == 64e-6


    def test_guard_sweeps_signed_arcs_and_blocks_unknown(self):
        grid = dict(width=80, height=80, resolution=.05, origin=(-2, -2, 0), data=[0] * 6400)
        footprint = ((-.145, -.14), (-.145, .14), (.2, .14), (.2, -.14))
        assert swept_clear((0, 0, 0), .4, 0, footprint, grid, 1.)
        grid['data'][40 * 80 + 49] = -1
        assert not swept_clear((0, 0, 0), .4, 0, footprint, grid, 1.)
        assert swept_clear((0, 0, 0), -.4, 0, footprint, grid, 1.)
        assert not swept_clear((1.9, 0, 0), .4, 0, footprint, grid, 1.)


    def test_shared_motion_limits_validate_footprint_and_braking(self):
        settings = dict(max_linear_mps=.8, max_angular_rps=.78, linear_acceleration_mps2=.4,
                        angular_acceleration_rps2=1., linear_braking_mps2=.2,
                        footprint=[[-.1, -.1], [-.1, .1], [.2, .1], [.2, -.1]])
        limits = MotionLimits(settings)
        assert limits.horizon((.8, 0, 0), .5) == 4.5
        self.assertAlmostEqual(limits.output((.8, .78), (0, 0), .1)[0], .04)
        self.assertAlmostEqual(limits.output((.8, .78), (0, 0), .1)[1], .1)
        settings['footprint'][2] = [-.05, 0]
        with self.assertRaisesRegex(ValueError, 'convex'):
            MotionLimits(settings)


    def test_servo_reply_never_blocks_motor_loop_and_echo_is_not_position(self):
        tx = []
        servo = ServoMailbox(ack_timeout=.12)
        servo.capability(1)
        request = servo.submit(1, 1750, 1.0)
        servo.tick(1.01, lambda payload: tx.append(payload))
        assert tx == [struct.pack('<H', 1750)]
        servo.tick(1.02, lambda payload: tx.append(payload))
        assert not request.done()
        servo.acknowledge(struct.pack('<H', 1500))
        assert not request.done()
        servo.acknowledge(struct.pack('<H', 1750))
        assert request.result() == (True, True, 'MCU echoed the requested pulse')
        assert servo.submit(2, 1750, 1.02).result()[0] is False
        assert servo.submit(1, 400, 1.02).result()[0] is False


    def test_servo_missing_echo_cannot_acknowledge_a_retry(self):
        servo = ServoMailbox(ack_timeout=.12)
        servo.capability(1)
        initial = servo.submit(1, 1750, 1.0)
        servo.tick(1.01, lambda _payload: None)
        servo.tick(1.15, lambda _payload: None)
        assert initial.result()[:2] == (True, False)
        retried = servo.submit(1, 1750, 1.16)
        servo.tick(1.17, lambda _payload: None)
        servo.acknowledge(struct.pack('<H', 1750))
        assert retried.result()[:2] == (True, False)

    def test_gamepad_requires_fresh_live_axis_events_and_stop_dominates(self):
        from slicer_base.gamepad import Gamepad
        import yaml

        mapping = Path(__file__).resolve().parents[1] / "src/slicer_base/config/gamepad.yaml"
        state = Gamepad(yaml.safe_load(mapping.read_text()))
        state.event(0x82, 1, -32767, 1.0)
        self.assertEqual(state.output(1.02), (False, (0.0, 0.0)))
        state.event(2, 1, -32767, 1.1)
        self.assertEqual(state.output(1.12), (True, (0.1, 0.0)))
        self.assertEqual(state.output(1.36), (False, (0.0, 0.0)))
        state.event(1, 7, 1, 1.37)
        self.assertEqual(state.output(1.38), (False, (0.0, 0.0)))
        state.event(2, 2, -32767, 1.39)
        self.assertEqual(state.output(1.4), (True, (0.0, 0.1)))
        self.assertTrue(state.event(1, 1, 1, 1.41))
        state.event(2, 2, -32767, 1.42)
        self.assertEqual(state.output(1.43), (False, (0.0, 0.0)))
