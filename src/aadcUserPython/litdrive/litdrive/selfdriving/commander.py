from .state import Car
from .enums import *


class Commander:
    def __init__(self, car: Car, lock, config: dict):
        self._car = car
        self._lock = lock
        self.out_speed = 0
        self.out_trajectories = (0, 1, 2, 3, 4, 1, 2, 3, 4, 0, 1, True)

    def decide(self):
        """
        Decide about the next outputs.
        """

        with self._lock:
            if self._car.THREAD_running and not self._car.running_previous:
                # start signal
                self._car.planner.initPlannerSettings(self._car.position["f32x"],
                                                      self._car.position["f32y"],
                                                      self._car.position["f32heading"])
                # send maneuvers
                for maneuver in self._car.THREAD_maneuvers:
                    mapped = _map_maneuver_state(maneuver.action)
                    mapped = [e for e in mapped if e != ManeuverState.INVALID]
                    self._car.planner.addManeuver(mapped)

        self.out_speed = 0.5


def _map_maneuver_state(m):
    if m == ManeuverAction.Undefined:
        return ManeuverState.INVALID
    elif m == ManeuverAction.Left:
        return ManeuverState.LEFT
    elif m == ManeuverAction.Right:
        return ManeuverState.RIGHT
    elif m == ManeuverAction.Straight:
        return ManeuverState.STRAIGHT
    elif m == ManeuverAction.ParallelParking:
        return ManeuverState.INVALID
    elif m == ManeuverAction.ParallelParking:
        return ManeuverState.INVALID
    elif m == ManeuverAction.PullOutLeft:
        return ManeuverState.INVALID
    elif m == ManeuverAction.PullOutRight:
        return ManeuverState.INVALID
    elif m == ManeuverAction.MergeLeft:
        return ManeuverState.MERGE
    elif m == ManeuverAction.MergeRight:
        return ManeuverState.MERGE
