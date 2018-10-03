import cv2
import numpy as np
import tensorflow as tf

from .predict_boxes_yolo_v1 import postprocess

image_path = "/home/mohamed/tensorflow/sample_img/car_img_resized.jpg"
frozen_network_path = "/home/mohamed/tensorflow/network/frozen-yolo-tiny-aadc.pb"


def load_graph(frozen_graph_filename):
    # We load the protobuf file from the disk and parse it to retrieve the
    # unserialized graph_def
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    # Then, we import the graph_def into a new Graph and returns it
    with tf.Graph().as_default() as graph:
        # The name var will prefix every op/nodes in your graph
        # Since we load everything in a new graph, this is not needed
        tf.import_graph_def(graph_def)
    return graph


def resize_input(im):
    h, w, c = [448, 448, 3]
    imsz = cv2.resize(im, (w, h))
    imsz = imsz / 255.
    imsz = imsz[:,:,::-1]
    return imsz


# We use our "load_graph" function
graph = load_graph(frozen_network_path) #newest frozen yolo trained on 28/9/2018 optimize_inference

image = cv2.imread(image_path)
image_resized = resize_input(image)
np_final = np.expand_dims(image_resized, axis=0)

x = graph.get_tensor_by_name('import/input:0')
y = graph.get_tensor_by_name('import/output:0')

#We launch a Session
with tf.Session(graph=graph) as sess:
    y_out = sess.run(y, feed_dict={
        x: np_final
    })

    # get boxes
    boxes = postprocess(y_out[0])
    # unpack boxes
    for b in boxes:
        left, right, top, bot, mess, max_indx, confidence = b
        print(left, right, top, bot, mess, max_indx, confidence)
