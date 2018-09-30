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


def postprocess(net_out, im, save = True):
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

    # meta = {'labels': ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor'], 'name': 'yolo-tiny', 'out_size': 1470, 'type': '[detection]', 'colors': [(254.0, 254.0, 254), (239.88888888888889, 211.66666666666669, 127), (225.77777777777777, 169.33333333333334, 0), (211.66666666666669, 127.0, 254), (197.55555555555557, 84.66666666666667, 127), (183.44444444444443, 42.33333333333332, 0), (169.33333333333334, 0.0, 254), (155.22222222222223, -42.33333333333335, 127), (141.11111111111111, -84.66666666666664, 0), (127.0, 254.0, 254), (112.88888888888889, 211.66666666666669, 127), (98.77777777777777, 169.33333333333334, 0), (84.66666666666667, 127.0, 254), (70.55555555555556, 84.66666666666667, 127), (56.44444444444444, 42.33333333333332, 0), (42.33333333333332, 0.0, 254), (28.222222222222236, -42.33333333333335, 127), (14.111111111111118, -84.66666666666664, 0), (0.0, 254.0, 254), (-14.111111111111118, 211.66666666666669, 127)], 'jitter': 0.2, 'sqrt': 1, 'side': 7, 'class_scale': 1, 'coord_scale': 5, 'model': 'cfg/v1/yolo-tiny.cfg', 'coords': 4, 'inp_size': [448, 448, 3], 'classes': 20, 'noobject_scale': 0.5, 'num': 2, 'net': {'width': 448, 'batch': 64, 'type': '[net]', 'scales': '5,5,2,2,.1,.1', 'decay': 0.0005, 'policy': 'steps', 'max_batches': 40000, 'channels': 3, 'height': 448, 'steps': '20,40,60,80,20000,30000', 'learning_rate': 0.0001, 'momentum': 0.9, 'subdivisions': 64}, 'softmax': 0, 'object_scale': 1, 'rescore': 1}

    flags = {'load': 35875, 'trainer': 'rmsprop', 'summary': '', 'momentum': 0.0, 'threshold': -0.1, 'saveVideo': False, 'keep': 20, 'queue': 1, 'save': 2000, 'demo': '', 'verbalise': True, 'gpu': 0.0, 'labels': 'labels-TY-aadc.txt', 'config': './cfg/', 'lr': 1e-05, 'pbLoad': '', 'metaLoad': '', 'train': False, 'gpuName': '/gpu:0', 'savepb': False, 'backup': './ckpt/', 'annotation': '../pascal/VOCdevkit/ANN/', 'batch': 16, 'epoch': 1000, 'model': 'cfg/v1/yolo-tiny-aadc.cfg', 'imgdir': './sample_img/', 'binary': './bin/', 'json': False, 'dataset': '../pascal/VOCdevkit/IMG/'}
    # flags =  {'keep': 20, 'labels': 'labels.txt', 'imgdir': 'sample_img/', 'trainer': 'rmsprop', 'verbalise': True, 'summary': '', 'threshold': -0.1, 'config': './cfg/', 'lr': 1e-05, 'load': '', 'train': False, 'model': '', 'annotation': '../pascal/VOCdevkit/ANN/', 'demo': '', 'epoch': 1000, 'json': False, 'momentum': 0.0, 'queue': 1, 'batch': 16, 'pbLoad': 'built_graph_clean/yolo-tiny.pb', 'savepb': False, 'backup': './ckpt/', 'save': 2000, 'gpuName': '/gpu:0', 'dataset': '../pascal/VOCdevkit/IMG/', 'gpu': 0.0, 'binary': './bin/', 'metaLoad': 'built_graph_clean/yolo-tiny.meta', 'saveVideo': False}
    threshold = 0.1
    boxes = findboxes(meta, net_out, threshold)

    # return boxes
    if type(im) is not np.ndarray:
        imgcv = cv2.imread(im)
    else: imgcv = im

    h, w, _ = imgcv.shape
    resultsForJSON = []
    resultsForDecision = []
    for b in boxes:
        boxResults = process_box(meta, b, h, w, threshold)

        if boxResults is None:
            continue

        resultsForDecision.append(boxResults)

    #     left, right, top, bot, mess, max_indx, confidence = boxResults
    #     thick = int((h + w) // 300)
    #     if flags['json']:
    #         resultsForJSON.append({"label": mess, "confidence": float('%.2f' % confidence), "topleft": {"x": left, "y": top}, "bottomright": {"x": right, "y": bot}})
    #         continue
    #
    #     cv2.rectangle(imgcv,
    #         (left, top), (right, bot),
    #         meta['colors'][max_indx], thick)
    #     cv2.putText(
    #         imgcv, mess, (left, top - 12),
    #         0, 1e-3 * h, meta['colors'][max_indx],
    #         thick // 3)
    #
    #
    # if not save: return imgcv
    #
    # outfolder = os.path.join(flags['imgdir'], 'out')
    # img_name = os.path.join(outfolder, "test.jpg")
    # if flags['json']:
    #     textJSON = json.dumps(resultsForJSON)
    #     textFile = os.path.splitext(img_name)[0] + ".json"
    #     with open(textFile, 'w') as f:
    #         f.write(textJSON)
    #     return
    #
    # cv2.imwrite(img_name, imgcv)

    return resultsForDecision
