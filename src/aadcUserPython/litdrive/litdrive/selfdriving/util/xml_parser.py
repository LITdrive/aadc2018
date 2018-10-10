from xml.etree import ElementTree
from collections import namedtuple

from ..enums import *

RoadSign = namedtuple('RoadSign', ["id", "x", "y", "radius", "direction", "type", "init"])
ParkingSpace = namedtuple('ParkingSpace', ["id", "x", "y", "status", "direction"])
Maneuver = namedtuple('Maneuver', ["id", "action", "extra"])


def parse_roadsigns(path: str):
    tree = ElementTree.parse(path)
    signs = []
    parking = []
    for e in tree.getroot():
        if e.tag == "roadSign":
            id_ = int(e.attrib["id"])
            x_ = float(e.attrib["x"])
            y_ = float(e.attrib["y"])
            radius_ = float(e.attrib["radius"])
            direction_ = int(e.attrib["direction"])
            type_ = _map_roadsign_type(id_)
            init_ = "init" in e.attrib and e.attrib["init"].isnumeric() and int(e.attrib["init"]) == 1
            rs = RoadSign(id_, x_, y_, radius_, direction_, type_, init_)
            signs.append(rs)
        elif e.tag == "parkingSpace":
            id_ = int(e.attrib["id"])
            x_ = float(e.attrib["x"])
            y_ = float(e.attrib["y"])
            status_ = int(e.attrib["status"])
            direction_ = int(e.attrib["direction"])
            ps = ParkingSpace(id_, x_, y_, status_, direction_)
            parking.append(ps)

    return signs, parking


def parse_maneuver(path: str):
    print(path)
    tree = ElementTree.parse(path)
    sectors = []
    for s in tree.getroot():
        # sid = s.attrib['id']
        maneuvers = []
        for m in s:
            mid_ = m.attrib['id']
            action_ = _map_maneuver_string(m.attrib['action'])
            extra_ = int(m.attrib['extra']) if ('extra' in m.attrib and m.attrib['extra'].isnumeric()) else None
            man = Maneuver(mid_, action_, extra_)
            maneuvers.append(man)
        sectors.append(maneuvers)
    return sectors


def _map_maneuver_string(s):
    if s == "left":
        return ManeuverAction.Left
    elif s == "right":
        return ManeuverAction.Right
    elif s == "straight":
        return ManeuverAction.Straight
    elif s == "parallel_parking":
        return ManeuverAction.ParallelParking
    elif s == "cross_parking":
        return ManeuverAction.CrossParking
    elif s == "pull_out_left":
        return ManeuverAction.PullOutLeft
    elif s == "pull_out_right":
        return ManeuverAction.PullOutRight
    elif s == "merge_left":
        return ManeuverAction.MergeLeft
    elif s == "merge_right":
        return ManeuverAction.MergeRight
    else:
        return ManeuverAction.Undefined


def _map_roadsign_type(i):
    if 0 <= i <= 15:
        return RoadSignType(i)
    else:
        return RoadSignType.Undefined
