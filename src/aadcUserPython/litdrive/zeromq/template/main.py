#########################################################################################
# PLEASE COPY THE ENCLOSING PYTHON PACKAGE 'template' AND CREATE YOUR OWN PYTHON FILTER #
#########################################################################################

import sys

# workaround that will find the packages in script mode
sys.path.append(r'../../')

import json
import time
import matplotlib.pyplot as plt

from zeromq.util.server import *

IMAGE_HEIGHT = 960
IMAGE_WIDTH = 1280

# open a server for the filter
zmq = ZmqServer("tcp://*:5555",
                ["tSignalValue", "tBoolSignalValue", "tWheelData",
                 "tInerMeasUnitData", "tUltrasonicStruct", "tVoltageStruct",
                 ("image1", IMAGE_HEIGHT, IMAGE_WIDTH)],
                ["tSignalValue", "tBoolSignalValue"])

# get an empty interactive view for the image
plt.ion()
plt.figure(figsize=(10, 10))
view = plt.imshow(np.ones((IMAGE_HEIGHT, IMAGE_WIDTH, 3)))


def process(signal, bool_signal, wheel, imu, us, voltage, image):
    # pretty-print the dictionaries
    print(json.dumps([signal, bool_signal, wheel, imu, us, voltage], indent=2))

    # update image
    view.set_data(image)
    plt.draw()
    plt.pause(.01)

    # you can omit individual outputs by sending None
    # return signal_out, None

    # you can also send an empty reply which will not trigger any outputs
    # return None

    # or you just send everything with tuples
    signal_out = (1337, 42)
    bool_out = (7331, True)
    return signal_out, bool_out


try:
    zmq.connect()
    zmq.run(process, return_dict=True)
finally:
    zmq.disconnect()
