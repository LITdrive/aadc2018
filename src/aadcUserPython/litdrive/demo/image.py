######################################################
# PLEASE COPY THIS TEMPLATE AND MAKE YOUR OWN FILTER #
######################################################

import numpy as np
import matplotlib.pyplot as plt

from zeromq.server import ZmqServer

IMAGE_HEIGHT = 960
IMAGE_WIDTH = 1280

# open a server for the filter
zmq = ZmqServer("tcp://*:5555", [("front", IMAGE_HEIGHT, IMAGE_WIDTH)])

# get an empty interactive view for the image
plt.ion()
plt.figure(figsize=(10, 10))
view = plt.imshow(np.ones((IMAGE_HEIGHT, IMAGE_WIDTH, 3)))


def process(image):
    view.set_data(image)
    plt.draw()
    plt.pause(.01)


try:
    zmq.connect()
    zmq.run(process)
finally:
    zmq.disconnect()
