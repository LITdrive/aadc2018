import enum

from .planning.planner import Planner


class GlobalState(enum.Enum):
    STOP = 1,
    INIT = 2,
    DRIVING = 3,
    RESTART = 4,
    ERROR = 5


class ManeuverState(enum.Enum):
    NONE = 1,
    STRAIGHT = 2,
    LEFT = 3,
    RIGHT = 4,
    MERGE_LEFT = 5,
    PARKING = 6


class Car:
    def __init__(self, config: dict):
        # initialize states
        self.state = GlobalState.INIT
        self.maneuver = ManeuverState.NONE

        # setup perception
        self.perception = Perception()

        # setup trajectory planner
        self.planner = Planner(config["roadFile"])

        # setup (empty) receptors
        self.siren_receptor = None
        self.roadsign_receptor = None


class Perception:
    def __init__(self):
        pass

    def is_the_cake_a_lie(self):
        return True
