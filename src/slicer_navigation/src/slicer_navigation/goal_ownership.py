from threading import Lock


class GoalOwnership:
    """Arbitrate the public goal and its currently running private subgoal."""

    def __init__(self):
        self._lock = Lock()
        self._goal = None
        self._canceled = False
        self._after_stamp = None
        self._target_stamp = None

    def accept(self, goal_id, valid):
        if not valid:
            return False
        with self._lock:
            self._goal = goal_id
            self._canceled = False
            self._after_stamp = None
        return True

    def arm(self, goal_id, after_stamp, target_stamp):
        with self._lock:
            if self._goal != goal_id or self._canceled:
                return False
            self._after_stamp = after_stamp
            self._target_stamp = target_stamp
            return True

    def cancel(self, goal_id):
        with self._lock:
            if self._goal != goal_id or self._canceled:
                return False
            self._canceled = True
            self._after_stamp = None
            return True

    def disarm(self, goal_id):
        with self._lock:
            if self._goal != goal_id:
                return False
            self._after_stamp = None
            return True

    def route(self, stamp, publish, target_stamp):
        with self._lock:
            if (self._after_stamp is not None and stamp > self._after_stamp
                    and target_stamp == self._target_stamp):
                return publish(self._goal)
        return None
