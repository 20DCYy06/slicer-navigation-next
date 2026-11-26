"""Interpret deliberate Linux joystick events without caching stale motion."""

from math import copysign


class Gamepad:
    def __init__(self, config):
        self.config = config
        self.axes = {}
        self.linear = config["linear_min"]
        self.angular = config["angular_min"]
        self.activity = float("-inf")
        self.estop = False

    def event(self, kind, number, value, now):
        config = self.config
        if kind & 0x80 or self.estop:  # JS_EVENT_INIT is a snapshot, not operator intent.
            return False
        if kind == 2 and number in (config["throttle_axis"], config["steering_axis"]):
            if now - self.activity >= config["event_timeout_s"]:
                self.axes.clear()
            self.axes[number] = max(-1., min(1., value / 32767.))
            self.activity = now
        elif kind == 1 and value:
            if number == config["stop_button"]:
                self.estop = True
                self.axes.clear()
                return True
            direction = int(number == config["up_button"]) - int(number == config["down_button"])
            self.linear = self.gear("linear", self.linear, direction)
            self.angular = self.gear("angular", self.angular, direction)
        return False

    def gear(self, name, value, direction):
        config = self.config
        return max(config[name + "_min"], min(config[name + "_max"],
                   value + direction * config[name + "_step"]))

    def axis(self, name):
        config = self.config
        value = self.axes.get(config[name + "_axis"], 0.)
        dead_zone = config["dead_zone"]
        if abs(value) <= dead_zone:
            return 0.
        return config[name + "_sign"] * copysign((abs(value) - dead_zone) / (1. - dead_zone), value)

    def output(self, now):
        if self.estop or not 0 <= now - self.activity < self.config["event_timeout_s"]:
            return False, (0., 0.)
        values = self.axis("throttle") * self.linear, self.axis("steering") * self.angular
        return any(values), values
