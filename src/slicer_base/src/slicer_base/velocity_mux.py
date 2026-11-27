"""Arbitrate fresh planar commands with explicit ownership transitions."""

from math import isfinite
from secrets import randbits


ZERO = (0.0, 0.0)


class VelocityMux:
    def __init__(self, timeout):
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError("command timeout must be positive")
        self.timeout = timeout
        # A previous process's authority requests remain stale after a restart.
        self.epoch = randbits(48)
        self.held = False
        self.manual = False
        self.estop = False
        self.online = False
        self.goal_id = ""
        self.since = float("-inf")
        self.since_stamp = 0
        self.slots = {}
        self.last_stamp = {}

    def _invalidate(self, now, stamp):
        self.epoch += 1
        self.goal_id = ""
        self.slots.clear()
        self.since, self.since_stamp = now, stamp

    def set_online(self, online, now, stamp=0):
        if self.online != online:
            self.online = online
            self._invalidate(now, stamp)

    def hold(self, now, stamp=0):
        if not self.held:
            self.held = True
            self._invalidate(now, stamp)
        return True, "autonomous and mission motion held"

    def release(self, now, stamp=0):
        if self.estop:
            return False, "emergency stop is latched"
        if not self.online:
            return False, "base is unavailable"
        if self.manual and self._fresh("teleop", now):
            return False, "manual command stream is still active"
        if self.held or self.manual:
            self.held = self.manual = False
            self._invalidate(now, stamp)
        return True, "hold released"

    def takeover(self, now, stamp=0):
        if not self.manual and not self.estop:
            self.manual = True
            self._invalidate(now, stamp)

    def emergency(self, active, now, stamp=0):
        if active and not self.estop:
            self.estop = self.held = True
            self._invalidate(now, stamp)

    def authorize(self, goal_id, enable, epoch, now, stamp=0):
        if not goal_id:
            return False, "goal ID required"
        if not enable:
            if goal_id != self.goal_id:
                return False, "goal has no navigation authority"
            self.goal_id = ""
            self.slots.pop("nav", None)
            return True, "navigation authority revoked"
        if epoch != self.epoch:
            return False, "control epoch changed"
        if not self.online or self.held or self.manual or self.estop:
            return False, "base is unavailable for autonomous motion"
        if self._fresh("mission", now):
            return False, "mission command stream owns motion"
        if goal_id != self.goal_id:
            self.goal_id = goal_id
            self.slots.pop("nav", None)
            self.since, self.since_stamp = now, stamp
        return True, "navigation authority accepted"

    def command(self, source, velocity, received, now, stamp, ros_now, goal_id=""):
        if not self.online or self.estop or received < self.since:
            return False
        if source == "teleop":
            if not self.manual:
                return False
        elif self.held or self.manual:
            return False
        if source == "nav" and (not goal_id or goal_id != self.goal_id):
            return False

        age = ros_now - stamp
        if (stamp <= max(self.since_stamp, self.last_stamp.get(source, 0))
                or not 0 <= age < self.timeout or not 0 <= now - received < self.timeout):
            return False
        self.last_stamp[source] = stamp
        if not all(isfinite(component) for component in velocity):
            self.slots.pop(source, None)
            return False
        if source == "mission" and not self._fresh("mission", now):
            # Mission preempts navigation. Only a newly authorized goal can run
            # once mission velocity has stopped arriving.
            if self.goal_id:
                self._invalidate(now, stamp)
        self.slots[source] = min(received + self.timeout, now + self.timeout - age), velocity
        return True

    def _fresh(self, source, now):
        return source in self.slots and now < self.slots[source][0]

    def select(self, now):
        if self.online and not self.estop:
            sources = ("teleop",) if self.manual else (() if self.held else ("mission", "nav"))
            for source in sources:
                if self._fresh(source, now):
                    return source, self.slots[source][1]
        return "zero", ZERO
