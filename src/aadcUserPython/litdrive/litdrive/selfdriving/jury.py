import json
import functools
import threading

from ..zeromq.server import ZmqServer


class JuryThread(threading.Thread):
    def __init__(self, address, car, lock):
        super().__init__()
        self._lock = lock
        self._car = car

        inputs = [
            "tBoolSignalValue",  # timer (TRIGGER)
            "tJuryStruct",  # jury (TRIGGER)
            "tBoolSignalValue",  # jury_data_update (TRIGGER)
            "tSignalValue",  # confidence_front
            "tSignalValue",  # confidence_rear
        ]
        outputs = [
            "tDriverStruct",  # driver
            "tSignalValue",  # position_mux
            "tBoolSignalValue"  # initial_localization
        ]

        self._server = ZmqServer(address, inputs, outputs)

    def run(self):
        print("- jury")
        func = functools.partial(JuryThread._process, self)
        try:
            self._server.connect()
            self._server.run(func, return_dict=True)
        finally:
            self._server.disconnect()

    @staticmethod
    def _process(parent, timer, jury, jury_update, conf_front, conf_rear):
        print("Jury processing ...")
        # debug output
        print(json.dumps([jury, jury_update, conf_front, conf_rear], indent=2))
