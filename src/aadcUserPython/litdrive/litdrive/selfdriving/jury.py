import json
import functools
import threading

from ..zeromq.server import ZmqServer
from .enums import *
from .util.xml_parser import parse_maneuver, parse_roadsigns


class JuryThread(threading.Thread):
    def __init__(self, address, car, lock, config):
        super().__init__()
        self._lock = lock
        self._car = car
        self._config = config

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

    def read_files(self):
        print("Reading road signs ...")
        parse_roadsigns(self._config["roadSignsFile"])
        print("Reading maneuver list ...")
        parse_roadsigns(self._config["maneuverListFile"])

    @staticmethod
    def _process(self, timer, jury, jury_update, conf_front, conf_rear):
        print("Jury processing ...")
        print(json.dumps([jury, jury_update, conf_front, conf_rear], indent=2))

        driver, position_mux, initial_localization = None, None, None

        if jury_update and jury_update["bValue"]:
            self.read_files()
            driver = (JuryCarState.StartUp.value, ManeuverAction.Undefined.value)

        if jury:
            action_id = JuryAction(jury["i16ActionID"])
            maneuver_entry = jury["i16ManeuverEntry"]

            if action_id == JuryAction.GetReady:
                print("Received GET READY signal.")
                driver = (JuryCarState.Ready.value, ManeuverAction.Undefined.value)
            elif action_id == JuryAction.Start:
                print("Received START signal.")
                pass
            elif action_id == JuryAction.Stop:
                print("Received STOP signal.")
                pass

        return driver, position_mux, initial_localization
