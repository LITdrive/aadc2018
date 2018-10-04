######################################################
# PLEASE COPY THIS TEMPLATE AND MAKE YOUR OWN FILTER #
######################################################

import numpy as np

from ..server import ZmqServer

TRAJECTORY_ARRAY_SIZE = 10
TRAJECTORY_NUM_FIELDS = 12


def process(_):
    trajectories = []

    # create a trajectory
    x = np.poly1d([1, 2, 3, 4])
    y = np.poly1d([1, 2, 3, 4])
    t1 = (42, *x.c, *y.c, 0, 1, False)

    # send up to 10 trajectories
    trajectories.append(t1)
    # trajectories.append(t2)
    # trajectories.append(t3)
    # ...

    # you can also send a single trajectory here
    trajectory = None

    print("Sending trajectories ...")

    return trajectory, build_trajectory_array_buffer(trajectories)


def build_trajectory_array_buffer(trajectories):
    if not trajectories or len(trajectories) == 0:
        return None

    assert len(trajectories) <= TRAJECTORY_ARRAY_SIZE
    buffer = ([[0] * TRAJECTORY_NUM_FIELDS] * TRAJECTORY_ARRAY_SIZE)

    for i, trajectory in enumerate(trajectories):
        buffer[i] = trajectory

    def _flatten(data):
        return [item for sublist in data for item in sublist]

    return [len(trajectories)] + _flatten(buffer)


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
