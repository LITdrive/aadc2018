import zmq
import numpy as np

from typing import Iterable

from zeromq.util.parser import pack, unpack, unpack_dict, FORMATS


class ZmqServer:
    def __init__(self, address: str, inputs: Iterable[str] = None, outputs: Iterable[str] = None):
        self._address = address
        self._inputs = inputs if inputs else []
        self._outputs = outputs if outputs else []

        # store image tuples in a separate list
        self._inputs_img = [(i, e) for i, e in enumerate(self._inputs) if isinstance(e, tuple)]
        self._images_available = any(self._inputs_img)

        # and remove them from the original one
        self._inputs = [e for e in self._inputs if not isinstance(e, tuple)]

        # cache format strings for fast access during packing and unpacking
        self._input_fmts = [FORMATS[e] for e in self._inputs]
        self._output_fmts = [FORMATS[e] for e in self._outputs]

        self._context = zmq.Context()
        self._socket = None
        self._process = None

    def connect(self):
        if not self._socket:
            self.disconnect()

        # open REP socket and do not wait (linger) at close time
        self._socket = self._context.socket(zmq.REP)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(self._address)

        print("Bound socket to {}".format(self._address))

    def disconnect(self):
        if self._socket:
            self._socket.disconnect()

    def set_process(self, process):
        self._process = process

    @staticmethod
    def _unpack_image(blob, height, width):
        if len(blob) == 0:
            return None

        if len(blob) != (3 * height * width):
            raise IOError("Image dimension mismatch. Expected image {}x{} (%d bytes) but we received %d bytes."
                          .format(height, width, (3 * height * width), len(blob)))

        return np.frombuffer(blob, dtype=np.uint8).reshape((height, width, 3))

    def run(self, process, return_dict):
        if not process:
            print("No process() method set.")

        while True:
            request_blob = self._socket.recv_multipart()

            # unpack all the images first
            images = []
            if self._images_available:
                for pin_index, (_, height, width) in self._inputs_img:
                    image = ZmqServer._unpack_image(request_blob[pin_index], height, width)
                    images.append(image)
                    del request_blob[pin_index]

            # fast-path unpacking
            request = [unpack_dict(req, dtype=dtype) for req, dtype in zip(request_blob, self._inputs)] if return_dict \
                else [unpack(req, fmt_str=fmt) for req, fmt in zip(request_blob, self._input_fmts)]

            # insert images into result
            if self._images_available:
                for image_index, (pin_index, _) in enumerate(self._inputs_img):
                    request.insert(pin_index, images.pop(image_index))

            reply = process(*request)
            reply_blob = [pack(rep, fmt_str=fmt)
                          for rep, fmt in zip(reply, self._output_fmts)] if reply else None

            # handle empty replies
            if reply:
                self._socket.send_multipart(reply_blob)
            else:
                self._socket.send(b'')
