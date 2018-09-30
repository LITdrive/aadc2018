import json
import argparse
import functools

from .state import Car
from .commander import Commander
from .receptors import *
from ..zeromq.server import ZmqServer


def main(socket: str, config: dict):
    print("LITdrive >>> Towards Autonomy.")

    car = Car(config)
    commander = Commander(car, config)

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
            "tJuryStruct",  # jury commands
            "tPosition",  # current position
            "tSignalValue",  # current speed
            "tInerMeasUnitData",  # imu
            "tUltrasonicStruct",  # ultrasonic
            "tRoadSignExt",  # road signs
            "tSignalValue"  # control feedback
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
                 jury, position, speed, imu, ultrasonic, road_signs, control_feedback):
        """
        Hand over new samples to our receptors and ask the commander for a decision
        :param car: A reference to the car model, which holds the receptors
        :return: The filter output
        """

        # debug output
        print(json.dumps([jury, position, speed, imu, ultrasonic, road_signs, control_feedback], indent=2))

        car.roadsign_receptor.update(position)
        car.siren_receptor.update()

        commander.decide()

        return (0, commander.out_speed), commander.out_trajectories


# python -m litdrive.selfdriving "tcp://*:5555"
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LITdrive Self-Driving AI")
    parser.add_argument('socket', type=str, default="tcp://*:5555",
                        help="The socket address for the ZeroMQ Server (e.g. 'tcp://*:5555')")
    args = parser.parse_args()

    # TODO: read configurations from command line
    main(args.socket, {"roadFile": None})
