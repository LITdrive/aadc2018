import enum


class ManeuverState(enum.IntEnum):
    NEXT = 0
    LEFT = 1
    STRAIGHT = 2
    RIGHT = 3
    MERGE = 4
    INVALID = 5
    END = 6
    MAX = 7


class JuryCarState(enum.IntEnum):
    Error = -1
    Ready = 0
    Running = 1
    Complete = 2
    StartUp = -2


class JuryAction(enum.IntEnum):
    Stop = -1
    GetReady = 0
    Start = 1


class ManeuverAction(enum.IntEnum):
    Undefined = 0
    Left = 1
    Right = 2
    Straight = 4
    ParallelParking = 5
    CrossParking = 6
    PullOutLeft = 7
    PullOutRight = 8
    MergeLeft = 9
    MergeRight = 10


class RoadSignType(enum.IntEnum):
    UnmarkedIntersection = 0
    StopAndGiveWay = 1
    ParkingArea = 2
    HaveWay = 3
    AheadOnly = 4
    GiveWay = 5
    PedestrianCrossing = 6
    Roundabout = 7
    NoOvertaking = 8
    NoEntryVehicularTraffic = 9
    TestCourseA9 = 10
    OneWayStreet = 11
    RoadWorks = 12
    KMH50 = 13
    KMH100 = 14
    Undefined = 99

class PlannerState(enum.IntEnum):
    INVALID=0
    ERROR=1
    NO_TRAJECTORIES=2
    FORWARD_NORMAL=3
    FORWARD_OVERTAKE=4
    BACKWARDS_NORMAL=5
    BACKWARDS_OVERTAKE=6
    FORWARD_PARK=7
    BACKWARD_PARK=8
    MAX=9

class PlannerErrors(enum.IntEnum):
    NONE=0
    INVALID_DECISION=1
    OUT_OF_DECISIONS=2