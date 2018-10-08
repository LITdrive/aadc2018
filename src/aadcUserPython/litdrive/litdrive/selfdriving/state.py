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

    def is_roadworks_ahead(self):
        return Fals

    def is_in_tunnel(self):
        return False

    def get_valid_traffic_signs(self): #or just this
        #TODO for crossingins
        return False

    def is_zebra_crossing_ahead(self):
        #TODO
        return False

    def is_person_at_zebra_crossing(self):
        #TODO
        return False

    def is_person_present(self):
        #check image TODO
        return False

    def is_traffic_comming_from_right(self):
        #TODO
        return False

    def is_traffic_comming_from_left(self):
        #TODO
        # oncoming slow traffic
        return False

    def is_obstacle_in_trajectory(self):
        # carwidth = 0.3 meters ;)
        #TODO --> do not forgett to indicate -_> Blinker
        # should also consider driving backwards
        return False

    def is_active_emergency_car(self):
        #TODO --> yielding
        return False

    def is_at_intersection(self):
        # never stop at an intersection!!!!!
        #TODO
        return False

    def is_vehicle_coming_from_behind(self):
        #for merging scenario
        #TODO
        return False







