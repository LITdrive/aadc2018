import enum

from .enums import *
from .planning.planner import Planner


class Car:
    def __init__(self, config: dict):
        # initialize states
        self.maneuver = ManeuverState.INVALID

        # setup perception
        self.perception = Perception()

        # setup trajectory planner
        self.planner = Planner(config["pickledOpenDriveMap"])

        # setup (empty) receptors
        self.siren_receptor = None
        self.roadsign_receptor = None

        # parsed maneuver and roadsign lists
        self.THREAD_maneuvers = None
        self.THREAD_roadsigns = None

        # jury signals
        self.THREAD_jury_stop_signal = False
        self.THREAD_jury_maneuver_entry = -1

        # jury callback
        # TODO: continue here with sending data
        self.THREAD_jury_current_maneuver = -1

        self.running_previous = False
        self.THREAD_running = False

        self.position = None


class Perception:
    def __init__(self):
        pass

    def is_the_cake_a_lie(self):
        return True

    def is_roadworks_ahead(self):
        return False

    def is_in_tunnel(self):
        return False

    def get_valid_traffic_signs(self):  # or just this
        # TODO for crossingins
        return False

    def is_zebra_crossing_ahead(self):
        # TODO
        return False

    def is_person_at_zebra_crossing(self):
        # TODO
        return False

    def is_person_present(self):
        # check image TODO
        return False

    def is_traffic_comming_from_right(self):
        # TODO
        return False

    def is_traffic_comming_from_left(self):
        # TODO
        # oncoming slow traffic
        return False

    def is_obstacle_in_trajectory(self):
        # carwidth = 0.3 meters ;)
        # TODO --> do not forgett to indicate -_> Blinker
        # should also consider driving backwards
        return False

    def is_active_emergency_car(self):
        # TODO --> yielding
        return False

    def is_at_intersection(self):
        # never stop at an intersection!!!!!
        # TODO
        return False

    def is_ready_for_merge_lane(self):
        # for merging scenario
        # check if moving object is in area
        # TODO
        return False
