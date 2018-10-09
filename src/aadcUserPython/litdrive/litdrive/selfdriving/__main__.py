import functools
import threading

from .commander import Commander
from .jury import JuryThread
from .receptors import *
from .state import Car
from ..zeromq.server import ZmqServer


def main(socket: str, config: dict):
    print("LITdrive >>> Towards Autonomy.")

    # lock concurrent access to the state
    lock = threading.Lock()

    car = Car(config)
    commander = Commander(car, lock, config)

    # setup and start jury thread
    car.jury_receptor = JuryThread("tcp://*:5561", car, lock)
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
        ]
        outputs = [
            "tSignalValue",  # desired speed
            "tTrajectory"  # desired trajectory
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
                 controller_leverage, controller_feedback, siren, lidar_break):
        print("- sensor")

        # debug output
        # print(json.dumps([position, measured_speed, signs, lidar, ultrasonic, imu,
        #                  controller_leverage, controller_feedback, siren, lidar_break], indent=2))

        car.roadsign_receptor.update(position)
        car.siren_receptor.update()

        commander.decide()

        return (0, commander.out_speed), commander.out_trajectories


if __name__ == "__main__":
    main("tcp://*:5562", {"roadFile": None})
