import time
import json
import functools
import threading

from ..zeromq.server import ZmqServer
from .enums import *
from .util.xml_parser import parse_maneuver, parse_roadsigns


class LocalizationState(enum.Enum):
    Inactive = 0
    WaitForMarkerPos = 1
    WaitForFineLoc = 2
    Normal = 3


class JuryThread(threading.Thread):
    def __init__(self, address, car, lock, config):
        super().__init__()
        self._lock = lock
        self._car = car
        self._config = config

        self._last_maneuver = None

        self._received_files = False

        # 0815 state machine for initial localization
        self._localization_state = LocalizationState.Inactive
        self._localization_timer = None

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
        roadsigns = parse_roadsigns(self._config["roadSignsFile"])
        print(roadsigns)
        print("Reading maneuver list ...")
        maneuver = parse_roadsigns(self._config["maneuverListFile"])
        print(maneuver)
        self._received_files = True
        with self._lock:
            self._car.THREAD_maneuvers = maneuver
            self._car.THREAD_roadsigns = roadsigns

    @staticmethod
    def _process(self, timer, jury, jury_update, conf_front, conf_rear):
        driver, position_mux, initial_localization = None, None, None

        if jury_update and jury_update["bValue"]:
            self.read_files()

        if jury:
            action_id = JuryAction(jury["i16ActionID"])
            maneuver_entry = jury["i16ManeuverEntry"]

            if action_id == JuryAction.GetReady:
                print("Received GET READY signal. Waiting for Marker Positioning ...")
                initial_localization = (0, False)
                position_mux = (0, 2)
                driver = (JuryCarState.StartUp.value, -1)
                self._localization_state = LocalizationState.WaitForMarkerPos
                self._localization_timer = time.time()

            elif action_id == JuryAction.Start:
                print("Received START signal.")
                if self._localization_state != LocalizationState.Normal or \
                        not self._received_files:
                    print("ERROR: Initial Positioing not yet done OR didn't receive files yet!")
                    driver = (JuryCarState.Error.value, -1)
                else:
                    driver = (JuryCarState.Running, maneuver_entry)
                    with self._lock:
                        self._car.THREAD_running = True
                        self._car.THREAD_jury_maneuver_entry = maneuver_entry
                        self._car.THREAD_jury_current_maneuver = maneuver_entry
                        self._car.THREAD_jury_stop_signal = False
            elif action_id == JuryAction.Stop:
                print("Received STOP signal.")
                with self._lock:
                    self._car.THREAD_running = False
                    self._car.THREAD_jury_stop_signal = True

        # localization state machine
        if self._localization_state == LocalizationState.WaitForMarkerPos and \
                int(time.time() - self._localization_timer) > 1:
            print("Marker Positioning done. Waiting for Fine Localization ...")
            initial_localization = (0, True)
            position_mux = (0, 1)
            self._localization_state = LocalizationState.WaitForFineLoc
            self._localization_timer = time.time()
        elif self._localization_state == LocalizationState.WaitForFineLoc and \
                int(time.time() - self._localization_timer) > 4:
            print("Fine Localization done. Switching to normal operation.")
            initial_localization = (0, False)
            position_mux = (0, 0)
            driver = (JuryCarState.Ready.value, -1)
            self._localization_state = LocalizationState.Normal

        if self._localization_state == LocalizationState.Normal:
            with self._lock:
                if self._car.THREAD_running:
                    maneuver = self._car.THREAD_jury_current_maneuver
                    if maneuver != self._last_maneuver:
                        driver = (JuryCarState.Running, maneuver)
                    self._last_maneuver = maneuver
                else:
                    driver = (JuryCarState.Ready.value, -1)

        return driver, position_mux, initial_localization
