import numpy as np

from ..darkflow.cy_yolo_findboxes import yolo_box_constructor


def process_box(meta, b, h, w, threshold):
    max_indx = np.argmax(b.probs)
    max_prob = b.probs[max_indx]
    label = meta['labels'][max_indx]
    if max_prob > threshold:
        left = int((b.x - b.w / 2.) * w)
        right = int((b.x + b.w / 2.) * w)
        top = int((b.y - b.h / 2.) * h)
        bot = int((b.y + b.h / 2.) * h)
        if left < 0:  left = 0
        if right > w - 1: right = w - 1
        if top < 0:   top = 0
        if bot > h - 1:   bot = h - 1
        mess = '{}'.format(label)
        return (left, right, top, bot, mess, max_indx, max_prob)
    return None


def findboxes(meta, net_out, threshold):
    boxes = []
    boxes = yolo_box_constructor(meta, net_out, threshold)

    return boxes


def postprocess(net_out):
    """
	Takes net output, draw predictions, save to disk
	"""
    meta = {"net": {"type": "[net]", "batch": 64, "subdivisions": 8, "width": 416, "height": 416, "channels": 3, "momentum": 0.9, "decay": 0.0005, "angle": 0, "saturation": 1.5, "exposure": 1.5, "hue": 0.1, "learning_rate": 0.001, "max_batches": 40100, "policy": "steps", "steps": "-1,100,20000,30000", "scales": ".1,10,.1,.1"}, "type": "[region]", "anchors": [1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52], "bias_match": 1, "classes": 20, "coords": 4, "num": 5, "softmax": 1, "jitter": 0.2, "rescore": 1, "object_scale": 5, "noobject_scale": 1, "class_scale": 1, "coord_scale": 1, "absolute": 1, "thresh": 0.5, "random": 1, "model": "cfg/tiny-yolo-voc.cfg", "inp_size": [416, 416, 3], "out_size": [13, 13, 125], "name": "tiny-yolo-voc", "labels": ["aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"], "colors": [[254.0, 254.0, 254], [239.88888888888889, 211.66666666666669, 127], [225.77777777777777, 169.33333333333334, 0], [211.66666666666669, 127.0, 254], [197.55555555555557, 84.66666666666667, 127], [183.44444444444443, 42.33333333333332, 0], [169.33333333333334, 0.0, 254], [155.22222222222223, -42.33333333333335, 127], [141.11111111111111, -84.66666666666664, 0], [127.0, 254.0, 254], [112.88888888888889, 211.66666666666669, 127], [98.77777777777777, 169.33333333333334, 0], [84.66666666666667, 127.0, 254], [70.55555555555556, 84.66666666666667, 127], [56.44444444444444, 42.33333333333332, 0], [42.33333333333332, 0.0, 254], [28.222222222222236, -42.33333333333335, 127], [14.111111111111118, -84.66666666666664, 0], [0.0, 254.0, 254], [-14.111111111111118, 211.66666666666669, 127]]}

    threshold = 0.1
    boxes = findboxes(meta, net_out, threshold)

    h, w, = [416, 416]
    resultsForDecision = []

    for b in boxes:
        boxResults = process_box(meta, b, h, w, threshold)

        if boxResults is None:
            continue

        resultsForDecision.append(boxResults)

    return resultsForDecision