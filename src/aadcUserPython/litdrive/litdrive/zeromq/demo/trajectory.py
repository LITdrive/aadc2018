######################################################
# PLEASE COPY THIS TEMPLATE AND MAKE YOUR OWN FILTER #
######################################################

import numpy as np

from ..server import ZmqServer


def process(_):
    # use this for empty polys
    empty = [0] * 12

    # id, x, y, start, end, backwards
    id = 42
    x = np.poly1d([1, 2, 3, 4])
    y = np.poly1d([1, 2, 3, 4])
    start, end, backwards = 0, 1, False
    t1 = (id, *x.c, *y.c, start, end, backwards)

    print("Sending polynomial ...")

    trajectory = t1
    trajectory_array = (*t1, *empty, *empty, *empty, *empty)
    return trajectory, trajectory_array


if __name__ == "__main__":
    # open a server for the filter
    zmq = ZmqServer("tcp://*:5555",
                    ["tSignalValue"],
                    ["tTrajectory", "tTrajectoryArray"])

    try:
        zmq.connect()
        zmq.run(process, return_dict=True)
    finally:
        zmq.disconnect()
