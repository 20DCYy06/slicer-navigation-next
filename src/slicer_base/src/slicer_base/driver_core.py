"""Wheel counter, stop evidence and non-blocking C30D servo request handling."""

from concurrent.futures import Future
from math import isfinite
import struct
from threading import Lock


class WheelSample:
    def __init__(self, signs):
        self.signs = signs
        self.previous = None

    def update(self, sample):
        previous = self.previous
        self.previous = sample
        if previous is None:
            return None
        tick, counts = sample
        dt = ((tick - previous[0]) & 0xffffffff) * 1e-6
        if not 0 < dt <= 1:
            return None
        delta = [
            (((after - before + 2**31) % 2**32) - 2**31) * sign
            for before, after, sign in zip(previous[1], counts, self.signs)
        ]
        return delta, dt


class StopObservation:
    def __init__(self, linear_limit, angular_limit, window, max_age):
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.window = window
        self.max_age = max_age
        self.received = self.stamp = self.quiet_since = float("-inf")

    def update(self, now, stamp, linear, angular, wheels):
        if stamp <= self.stamp:
            return
        previous = self.received
        self.received, self.stamp = now, stamp
        if (abs(linear) > self.linear_limit or abs(angular) > self.angular_limit
                or any(abs(wheel) > self.linear_limit for wheel in wheels)):
            self.quiet_since = float("-inf")
        elif not isfinite(self.quiet_since) or now - previous > self.max_age:
            self.quiet_since = now

    def state(self, now):
        valid = 0 <= now - self.received <= self.max_age
        return valid, valid and self.received - self.quiet_since >= self.window and isfinite(self.quiet_since)

    def invalidate(self):
        self.received = self.stamp = self.quiet_since = float("-inf")


class ServoMailbox:
    """Post one pulse to the owner loop; only that loop sends or reads frames."""

    def __init__(self, ack_timeout, queue_timeout=.25):
        self.ack_timeout = ack_timeout
        self.queue_timeout = queue_timeout
        self.lock = Lock()
        self.channel = None
        self.pending = None
        self.in_flight = None
        self.closed = False
        self.unconfirmed_pulses = set()

    def capability(self, channel):
        with self.lock:
            self.channel = channel

    def submit(self, channel, pulse, now):
        result = Future()
        with self.lock:
            if self.closed:
                result.set_result((False, False, "driver unavailable"))
            elif self.channel is None:
                result.set_result((False, False, "servo capability unavailable"))
            elif channel != self.channel or not 500 <= pulse <= 2500:
                result.set_result((False, False, "channel or pulse outside verified servo capability"))
            elif self.pending is not None or self.in_flight is not None:
                result.set_result((False, False, "a servo request is pending"))
            else:
                self.pending = (result, pulse, now + self.queue_timeout)
        return result

    def tick(self, now, send):
        with self.lock:
            if self.in_flight is not None:
                result, pulse, deadline = self.in_flight
                if now >= deadline:
                    self.unconfirmed_pulses.add(pulse)
                    result.set_result((True, False, "pulse sent, MCU reply timed out"))
                    self.in_flight = None
            if self.pending is None:
                return
            result, pulse, deadline = self.pending
            self.pending = None
            if not result.set_running_or_notify_cancel():
                return
            if now >= deadline:
                result.set_result((False, False, "servo request expired before transmission"))
                return
            self.in_flight = (result, pulse, now + self.ack_timeout)

        # The hardware transport has its own bounded write. No reply is read
        # or waited for here, and the caller just sent its body command.
        try:
            send(struct.pack("<H", pulse))
        except Exception:
            with self.lock:
                self.in_flight = None
                result.set_result((False, False, "serial pulse transmission failed"))
            raise
        with self.lock:
            if pulse in self.unconfirmed_pulses:
                self.in_flight = None
                result.set_result((True, False, "pulse sent, an earlier identical echo is unresolved"))

    def acknowledge(self, payload):
        if len(payload) != 2:
            return
        (pulse,) = struct.unpack("<H", payload)
        with self.lock:
            if self.in_flight is None or pulse != self.in_flight[1] or pulse in self.unconfirmed_pulses:
                return
            result = self.in_flight[0]
            self.in_flight = None
            result.set_result((True, True, "MCU echoed the requested pulse"))

    def close(self):
        with self.lock:
            self.closed = True
            if self.pending is not None:
                result = self.pending[0]
                if result.set_running_or_notify_cancel():
                    result.set_result((False, False, "driver stopped before servo transmission"))
            if self.in_flight is not None:
                self.in_flight[0].set_result((True, False, "driver stopped while awaiting MCU reply"))
            self.pending = self.in_flight = None
