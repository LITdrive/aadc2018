######################################################
# PLEASE COPY THIS TEMPLATE AND MAKE YOUR OWN FILTER #
######################################################

import numpy as np

from ..server import ZmqServer
from ...util.findboxes.predict_boxes_yolo_v1 import postprocess


def process(yolo):
    if yolo:
        yolo_ = np.array(yolo, dtype=np.float32)

        # sort by last value (confidence)
        bounding_boxes = postprocess(yolo_)
        bounding_boxes = list(sorted(bounding_boxes, key=lambda box: -box[6]))

        print(bounding_boxes)

    return None


if __name__ == "__main__":
    # open a server for the filter
    zmq = ZmqServer("tcp://*:5555",
                    ["tYOLONetOutput"])

    try:
        zmq.connect()
        zmq.run(process, return_dict=False)
    finally:
        zmq.disconnect()
