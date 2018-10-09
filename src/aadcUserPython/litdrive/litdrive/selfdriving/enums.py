import enum

class GlobalState(enum.Enum):
    STOP = 1,
    INIT = 2,
    DRIVING = 3,
    RESTART = 4,
    ERROR = 5


class ManeuverState(enum.IntEnum):
    NEXT=0
    LEFT=1
    STRAIGHT=2
    RIGHT=3
    MERGE=4
    INVALID=5
    END=6
    MAX=7