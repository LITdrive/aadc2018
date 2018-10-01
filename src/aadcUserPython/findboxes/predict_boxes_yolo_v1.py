import numpy as np
import cv2
import os
import json
from darkflow_utils.cy_yolo_findboxes import yolo_box_constructor


def process_box(meta, b, h, w, threshold):
	max_indx = np.argmax(b.probs)
	max_prob = b.probs[max_indx]
	label = meta['labels'][max_indx]
	if max_prob > threshold:
		left  = int ((b.x - b.w/2.) * w)
		right = int ((b.x + b.w/2.) * w)
		top   = int ((b.y - b.h/2.) * h)
		bot   = int ((b.y + b.h/2.) * h)
		if left  < 0    :  left = 0
		if right > w - 1: right = w - 1
		if top   < 0    :   top = 0
		if bot   > h - 1:   bot = h - 1
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
    meta = {'side': 7, 'classes': 2, 'jitter': 0.2, 'num': 2, 'object_scale': 1, 'out_size': 588, 'inp_size': [448, 448, 3],
            'model': 'cfg/v1/yolo-tiny-aadc.cfg', 'colors': [(254.0, 254.0, 254), (222.25, 190.5, 127)], 'rescore': 1,
            'coords': 4, 'coord_scale': 5,
            'net': {'channels': 3, 'type': '[net]', 'batch': 64, 'momentum': 0.9, 'scales': '5,5,2,2,.1,.1',
                    'learning_rate': 0.0001, 'subdivisions': 64, 'policy': 'steps', 'max_batches': 40000, 'height': 448,
                    'decay': 0.0005, 'width': 448, 'steps': '20,40,60,80,20000,30000'}, 'name': 'yolo-tiny-aadc',
            'labels': ['person', 'car'], 'type': '[detection]', 'softmax': 0, 'sqrt': 1, 'class_scale': 1,
            'noobject_scale': 0.5}

    flags = {'load': 35875, 'trainer': 'rmsprop', 'summary': '', 'momentum': 0.0, 'threshold': -0.1, 'saveVideo': False, 'keep': 20, 'queue': 1, 'save': 2000, 'demo': '', 'verbalise': True, 'gpu': 0.0, 'labels': 'labels-TY-aadc.txt', 'config': './cfg/', 'lr': 1e-05, 'pbLoad': '', 'metaLoad': '', 'train': False, 'gpuName': '/gpu:0', 'savepb': False, 'backup': './ckpt/', 'annotation': '../pascal/VOCdevkit/ANN/', 'batch': 16, 'epoch': 1000, 'model': 'cfg/v1/yolo-tiny-aadc.cfg', 'imgdir': './sample_img/', 'binary': './bin/', 'json': False, 'dataset': '../pascal/VOCdevkit/IMG/'}

    threshold = 0.1
    boxes = findboxes(meta, net_out, threshold)

    h, w, = [448, 448]
    resultsForDecision = []

    for b in boxes:
        boxResults = process_box(meta, b, h, w, threshold)

        if boxResults is None:
            continue

        resultsForDecision.append(boxResults)

    return resultsForDecision
