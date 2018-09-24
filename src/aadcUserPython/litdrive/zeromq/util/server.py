import zmq

from typing import Iterable

from zeromq.util.parser import pack, unpack, unpack_dict, FORMATS


class ZmqServer:
    def __init__(self, address: str, inputs: Iterable[str] = None, outputs: Iterable[str] = None):
        self.address = address
        self.inputs = inputs if inputs else []
        self.outputs = outputs if outputs else []

        self._context = zmq.Context()
        self._socket = None
        self._process = None

        # cache format strings for fast access
        self._input_fmts = [FORMATS[e] for e in self.inputs]
        self._output_fmts = [FORMATS[e] for e in self.outputs]

    def connect(self):
        if not self._socket:
            self.disconnect()

        # open REP socket and do not wait (linger) at close time
        self._socket = self._context.socket(zmq.REP)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(self.address)

        print("Bound socket to {}".format(self.address))

    def disconnect(self):
        if self._socket:
            self._socket.disconnect()

    def set_process(self, process):
        self._process = process

    def run(self, process, return_dict):
        if not process:
            print("No process() method set.")

        while True:
            request_blob = self._socket.recv_multipart()
            request = [unpack_dict(req, dtype=dtype) for req, dtype in zip(request_blob, self.inputs)] if return_dict \
                else [unpack(req, fmt_str=fmt) for req, fmt in zip(request_blob, self._input_fmts)]

            reply = process(*request)
            reply_blob = [pack(rep, fmt_str=fmt)
                          for rep, fmt in zip(reply, self._output_fmts)] if reply else None

            # handle empty replies
            if reply:
                self._socket.send_multipart(reply_blob)
            else:
                self._socket.send(b'')
