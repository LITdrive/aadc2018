import functools
import threading

from .commander import Commander
from .jury import JuryThread
from .receptors import *
from .state import Car
from ..zeromq.server import ZmqServer

from os.path import dirname, join, abspath


def main(socket: str, config: dict):
    print("LITdrive >>> Towards Autonomy.")

    # lock concurrent access to the state
    lock = threading.Lock()

    car = Car(config)
    commander = Commander(car, lock, config)

    # setup and start jury thread
    car.jury_receptor = JuryThread("tcp://*:5561", car, lock, config)
    car.jury_receptor.start()

    # setup receptors
    car.siren_receptor = SirenReceptor(car.perception)
    car.roadsign_receptor = RoadSignReceptor(car.perception)

    # initialize receptors
    car.siren_receptor.init()
    car.roadsign_receptor.init()

    # run adtf filter
    server = DecisionServer(socket, car, commander)
    server.run()


class DecisionServer:
    def __init__(self, address: str, car: Car, commander: Commander):
        self._car = car
        self._commander = commander

        inputs = [
            "tBoolSignalValue",  # timer (TRIGGER)
            "tPosition",  # position
            "tSignalValue",  # measured_speed
            "tRoadSignExt",  # signs
            "tLaserScannerData",  # lidar
            "tUltrasonicStruct",  # ultrasonic
            "tInerMeasUnitData",  # imu
            "tPolynomPoint",  # controller_leverage
            "tPolynomPoint",  # controller_feedback
            "tBoolSignalValue",  # siren
            "tBoolSignalValue",  # lidar_break
            "tBoolSignalValue",  # lidarzone1
            "tBoolSignalValue",  # lidarzone2
            "tBoolSignalValue",  # lidarzone3
            "tBoolSignalValue",  # lidarzone4
            "tBoolSignalValue",  # lidarzone5
            "tBoolSignalValue",  # lidarzone6
            "tBoolSignalValue",  # lidarzone7
            "tBoolSignalValue",  # lidarzone8
            "tBoolSignalValue",  # lidarzone9
            "tBoolSignalValue",  # lidarzone10
            "tBoolSignalValue",  # lidarzone11
            "tBoolSignalValue",  # lidarzone12
            "tBoolSignalValue",  # lidarzone13
            "tBoolSignalValue",  # lidarzone14
            "tBoolSignalValue",  # lidarzone15
            "tBoolSignalValue",  # lidarzone16
            "tBoolSignalValue",  # lidarzone17
            "tBoolSignalValue",  # lidarzone18
            "tBoolSignalValue",  # lidarzone19
            "tBoolSignalValue"  # lidarzone20
        ]
        outputs = [
            "tSignalValue",  # desired speed
            "tTrajectory",  # desired trajectory
            "tBoolSignalValue",  # turn_signal_right
            "tBoolSignalValue",  # turn_signal_left
            "tBoolSignalValue",  # hazard_light
            "tBoolSignalValue",  # brake_light
            "tBoolSignalValue"  # reverse_light
        ]

        self._server = ZmqServer(address, inputs, outputs)

    def run(self):
        func = functools.partial(DecisionServer._process, self._car, self._commander)
        try:
            self._server.connect()
            self._server.run(func, return_dict=True)
        finally:
            self._server.disconnect()

    @staticmethod
    def _process(car: Car, commander: Commander,
                 timer, position, measured_speed, signs, lidar, ultrasonic, imu,
                 controller_leverage, controller_feedback, siren, lidar_break,
                 lz1, lz2, lz3, lz4, lz5, lz6, lz7, lz8, lz9, lz10,
                 lz11, lz12, lz13, lz14, lz15, lz16, lz17, lz18, lz19, lz20):
        # print("- sensor")
        print(position)
        # debug output
        # print(json.dumps([position, measured_speed, signs, lidar, ultrasonic, imu,
        #                  controller_leverage, controller_feedback, siren, lidar_break], indent=2))

        car.roadsign_receptor.update(position)
        car.siren_receptor.update()

        if position:
            car.position = position

        commander.decide()

        if siren:
            print("SIREN DETECTED")
        if lidar_break:
            print("LIDAR BREAK")

        jury_break = False
        with commander._lock:
            jury_break = car.THREAD_jury_stop_signal

        if jury_break:
            print("JURY BREAK")

        # emergency break
        with commander._lock:
            break_signal = siren or lidar_break or car.THREAD_jury_stop_signal

        # Should be called last, so that new decisions can already be taken into account in this call.
        out_trajectories = None
        if controller_leverage and controller_feedback:
            out_trajectories = car.planner.update(int(controller_feedback["id"]), int(controller_leverage["id"]),
                                                  float(controller_leverage["p"]))

        return (0, 0 if break_signal else commander.out_speed), out_trajectories


if __name__ == "__main__":
    main("tcp://*:5562", {
        "roadFile": None,
        "roadSignsFile":
            abspath(join(dirname(__file__), r'../../../../../configuration_files/jury/roadsigns.xml')),
        "maneuverListFile":
            abspath(join(dirname(__file__), r'../../../../../configuration_files/jury/maneuver.xml')),
        "pickledOpenDriveMap":
            abspath(join(dirname(__file__), r'../../../../../configuration_files/maps/qualifying_2018_litd.pickle'))
    })
