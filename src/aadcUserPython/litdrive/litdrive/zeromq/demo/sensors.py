######################################################
# PLEASE COPY THIS TEMPLATE AND MAKE YOUR OWN FILTER #
######################################################

import json

from ..server import ZmqServer


def process(signal, bool_signal, wheel, imu, lidar, us, voltage):
    # pretty-print the dictionaries
    print(json.dumps([signal, bool_signal, wheel, imu, lidar, us, voltage], indent=2))

    # you can omit individual outputs by sending None
    # return signal_out, None

    # you can also send an empty reply which will not trigger any outputs
    # return None

    # or you just send everything with tuples
    signal_out = (1337, 42)
    bool_out = (7331, True)
    return signal_out, bool_out


if __name__ == "__main__":
    # open a server for the filter
    zmq = ZmqServer("tcp://*:5555",
                    ["tSignalValue", "tBoolSignalValue", "tWheelData",
                     "tInerMeasUnitData", "tLaserScannerData", "tUltrasonicStruct", "tVoltageStruct"],
                    ["tSignalValue", "tBoolSignalValue"])

    try:
        zmq.connect()
        zmq.run(process, return_dict=True)
    finally:
        zmq.disconnect()
