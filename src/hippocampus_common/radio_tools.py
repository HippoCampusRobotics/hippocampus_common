from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals

import serial
import time
import re


class Configurator(object):

    EEPROM_PARAMETERS = dict(
        format=dict(register=0, readonly=True, options=None, default=None),
        serial_speed=dict(register=1,
                          readonly=False,
                          options=[2, 4, 9, 19, 38, 57, 115],
                          default=57),
        air_speed=dict(
            register=2,
            readonly=False,
            options=[2, 4, 8, 16, 19, 24, 32, 48, 64, 96, 128, 192, 250],
            default=64),
        netid=dict(register=3, readonly=False, options=list(range(0, 26)),
                   default=25),
        txpower=dict(register=4,
                     readonly=False,
                     options=[1, 2, 5, 8, 11, 14, 17, 20],
                     default=11),
        ecc=dict(register=5, readonly=False, options=[0, 1], default=0),
        mavlink=dict(register=6, readonly=False, options=[0, 1, 2], default=1),
        oppresend=dict(register=7, readonly=False, options=[0, 1], default=0),
        min_freq=dict(register=8,
                      readonly=False,
                      options=[433050],
                      default=433050),
        max_freq=dict(register=9,
                      readonly=False,
                      options=[434790],
                      default=434790),
        num_channels=dict(register=10,
                          readonly=False,
                          options=list(range(5, 51)),
                          default=20),
        duty_cycle=dict(register=11,
                        readonly=False,
                        options=list(range(10, 101)),
                        default=100),
        lbt_rssi=dict(register=12,
                      readonly=False,
                      options=list(range(0, 256)),
                      default=0),
        manchester=dict(register=13, readonly=False, options=[0, 1], default=0),
        rtscts=dict(register=14, readonly=False, options=[0, 1], default=0),
        nodeid=dict(register=15,
                    readonly=False,
                    options=list(range(0, 30)),
                    default=2),
        nodedestination=dict(register=16,
                             readonly=False,
                             options=list(range(0, 30)) + [65535],
                             default=65535),
        syncany=dict(register=17, readonly=False, options=[0, 1], default=0),
        nodecount=dict(register=18,
                       readonly=False,
                       options=list(range(2, 31)),
                       default=5),
    )

    AT_COMMAND = dict(write_eeprom="AT&W\r\n",
                      exit="ATO\r\n",
                      reboot="ATZ\r\n",
                      show_eeprom="ATI5\r\n",
                      show_version="ATI\r\n")

    def __init__(self, port=None, baud=57600):
        if port:
            self.port = serial.Serial(port, baud)
        else:
            self.port = serial.Serial()
        self.port.timeout = 0.2
        self._desired_params = dict()
        self._current_params = dict()

    @staticmethod
    def get_default_params(writable_only=True):
        params = dict()
        for param in Configurator.EEPROM_PARAMETERS:
            if Configurator.EEPROM_PARAMETERS[param]["readonly"]:
                continue
            params[param] = Configurator.EEPROM_PARAMETERS[param]["default"]
        return params

    def _parse_ok(self, bytes):
        if len(bytes) < 4:
            return False
        if b"OK" in bytes[-4:]:
            return True
        return False

    def read_until_ok(self, retries=3):
        success = False
        while True:
            bytes = self.port.readline()
            print(bytes)
            if len(bytes) > 0:
                success = self._parse_ok(bytes)
            else:
                retries -= 1
            if retries <= 0 or success:
                return success

    def set_desired_params(self, params):
        new_params = dict()
        for key in params:
            if key not in self.EEPROM_PARAMETERS:
                print("Skipping unknown paramter '{}'".format(key))
                continue
            new_params[key] = params[key]
        self._desired_params = new_params

    def write_params(self, params=None):
        failed_params = []
        if params is None:
            params = self._desired_params
        if self._read_param_format() == 27:
            current_nodecount = self._current_params["nodecount"]
            current_nodeid = self._current_params["nodeid"]
            desired_nodecount = params["nodecount"]
            desired_nodeid = params["nodeid"]
            if current_nodeid >= desired_nodecount:
                self._write_param(self.EEPROM_PARAMETERS["nodeid"]["register"],
                                  desired_nodeid)
            if desired_nodeid >= current_nodecount:
                self._write_param(
                    self.EEPROM_PARAMETERS["nodecount"]["register"],
                    desired_nodecount)
        for param in params:
            if self.EEPROM_PARAMETERS[param]["readonly"]:
                continue
            register = self.EEPROM_PARAMETERS[param]["register"]
            if not (self._write_param(register, params[param])):
                failed_params.append(register)
        return failed_params

    def write_eeprom(self):
        self.port.write(self.AT_COMMAND["write_eeprom"].encode())
        while True:
            line = self.port.readline().decode("utf-8")
            if "OK" in line:
                break
            if line == "":
                print("Failed to write eeprom.")
                return False

    def _write_param(self, register, value):
        data = "ATS{}={}\r\n".format(register, value).encode()
        self.port.write(data)

        while True:
            line = self.port.readline().decode("utf-8")
            if "OK" in line:
                break
            if line == "":
                return False
        return True

    def write_param(self, name, value):
        if name not in self.EEPROM_PARAMETERS:
            return False
        if self.EEPROM_PARAMETERS[name]["readonly"]:
            return True
        register = self.EEPROM_PARAMETERS[name]["register"]
        return self._write_param(register, value)

    def check_param(self, name, value):
        if name not in self.EEPROM_PARAMETERS:
            return False
        if self.EEPROM_PARAMETERS[name]["options"] is None:
            return True
        else:
            return int(value) in self.EEPROM_PARAMETERS[name]["options"]

    def _parse_param(self):
        value = None
        while True:
            answer = self.port.readline().decode("utf-8")
            if not answer:
                break
            try:
                value = int(answer.split()[-1])
            except ValueError:
                pass
            else:
                break
        return value

    def read_param(self, name):
        if name not in self.EEPROM_PARAMETERS:
            return None
        register = self.EEPROM_PARAMETERS[name]["register"]
        self.port.reset_input_buffer()
        self.port.write("ATS{}?\r\n".format(register).encode())
        self.port.flush()
        return self._parse_param()

    def enter_at_mode(self, retries=5):
        success = False
        while True:
            print("Entering AT mode...")
            for i in range(2):
                self.exit_at_mode()
            time.sleep(1.05)
            self.port.write(b"+++")
            self.port.flush()
            self.port.reset_input_buffer()
            time.sleep(1.05)
            success = self.read_until_ok()
            if success:
                self.port.flushInput()
                return success
            print("Could not enter AT mode. Keep trying...")
            time.sleep(1.5)
            retries -= 1
            print("{} retries left".format(retries))
            if retries == 0:
                self.port.flushInput()
                return False

    def exit_at_mode(self):
        try:
            self.port.write(self.AT_COMMAND["exit"].encode())
        except serial.serialutil.SerialException:
            return False
        return True

    def reboot(self):
        try:
            self.port.write(self.AT_COMMAND["reboot"].encode())
        except serial.serialutil.SerialException:
            return False
        return True

    def show_version(self):
        self.port.reset_input_buffer()
        self.port.write(self.AT_COMMAND["show_version"].encode())
        version = ""
        while True:
            bytes = self.port.readline()
            print(bytes)
            answer = bytes.decode("utf-8")
            if not answer:
                break
            if answer == self.AT_COMMAND["show_version"]:
                continue
            version = answer
            break
        return version

    def read_parameters_from_eeprom(self):
        pattern = self._get_param_pattern()
        if pattern is None:
            return None

        parameters = {}
        self.port.write(self.AT_COMMAND["show_eeprom"].encode())
        while True:
            answer = self.port.readline().decode("utf-8")
            answer = answer.lstrip().rstrip()
            answer = answer.replace(" ", "")
            if not answer:
                break
            match = pattern.search(answer)
            if match is not None:
                parameters[match.group(1).lower()] = int(match.group(2))
        self._current_params = parameters
        return parameters

    def _read_param_format(self):
        self.port.reset_input_buffer()
        self.port.write("ATS0?\r\n".encode())
        self.port.flush()
        time.sleep(1.0)
        param_format = None
        while True:
            answer = self.port.readline().decode("utf-8")
            if not answer:
                break
            try:
                param_format = int(answer.split()[-1])
            except ValueError:
                pass
            else:
                break
        return param_format

    def _get_param_pattern(self):
        param_format = self._read_param_format()
        pattern = None
        if param_format == 26 or param_format == 27:
            pattern = re.compile(r"[0-9]+:([a-zA-z_]*)=([0-9]+)")
        else:
            print("Paramter format '{}' not supported. Please implement it!".
                  format(param_format))
        return pattern


def main():
    c = Configurator()
    if not c.enter_at_mode():
        print("Failed to enter AT mode.")
        exit(1)
    print(c.get_version())

    c.read_parameters_from_eeprom()
    c.set_desired_params(c.EEPROM_PARAMETERS)
    c.write_params()
    c.read_parameters_from_eeprom()
    c.write_eeprom()
    c.reboot()
    time.sleep(2.0)
    if not c.enter_at_mode():
        print("Failed to enter AT mode.")
        exit(1)
    c.read_parameters_from_eeprom()


if __name__ == "__main__":
    main()
