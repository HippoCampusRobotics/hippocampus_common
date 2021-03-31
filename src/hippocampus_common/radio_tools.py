from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals

import serial
import time
import re
from tabulate import tabulate


class BaseConfigurator(object):

    EEPROM_PARAMETERS = dict(serial_speed=57,
                             air_speed=64,
                             netid=12,
                             txpower=20,
                             mavlink=1,
                             oppresend=0,
                             min_freq=433050,
                             max_freq=434790,
                             num_channels=10,
                             duty_cycle=100,
                             lbt_rssi=0,
                             manchester=0,
                             rtscts=0)

    AT_EEPROM = dict(
        serial_speed="ATS1",
        air_speed="ATS2",
        netid="ATS3",
        txpower="ATS4",
        mavlink="ATS6",
        oppresend="ATS7",
        min_freq="ATS8",
        max_freq="ATS9",
        num_channels="ATS10",
        duty_cycle="ATS11",
        lbt_rssi="ATS12",
        manchester="ATS13",
        rtscts="ATS14",
    )

    AT_COMMAND = dict(write_eeprom="AT&W\r\n",
                      exit="ATO\r\n",
                      reboot="ATZ\r\n",
                      show_eeprom="ATI5\r\n",
                      show_version="ATI\r\n")

    def __init__(self, port="/dev/ttyUSB0", baud=57600):
        self._port = serial.Serial(port, baud, timeout=3)
        self._desired_params = dict()
        self._current_params = dict()

    def _parse_ok(self, bytes):
        if len(bytes) < 4:
            return False
        if b"OK" in bytes[-4:]:
            return True
        return False

    def read_until_ok(self, retries=3):
        success = False
        while True:
            bytes = self._port.readline()
            if len(bytes) > 0:
                success = self._parse_ok(bytes)
            else:
                retries -= 1
            if retries <= 0 or success:
                return success

    def set_desired_params(self, params):
        new_params = dict()
        for key in params:
            if not key in self.AT_EEPROM:
                print("Skipping unknown paramter '{}'".format(key))
                continue
            new_params[key] = params[key]
        self._desired_params = new_params

    def write_params(self):
        for key in self._desired_params:
            if self._desired_params[key] != self._current_params[key]:
                self._write_param(key, self._desired_params[key])

    def write_eeprom(self):
        self._port.write(self.AT_COMMAND["write_eeprom"].encode())
        while True:
            line = self._port.readline().decode("utf-8")
            if "OK" in line:
                break
            if line == "":
                print("Failed to write eeprom.")
                return False

    def reboot(self):
        self._port.write(self.AT_COMMAND["reboot"].encode())

    def _write_param(self, param_name, value):
        data = "{}={}\r\n".format(self.AT_EEPROM[param_name], value).encode()
        self._port.write(data)

        while True:
            line = self._port.readline().decode("utf-8")
            if "OK" in line:
                break
            if line == "":
                print("Failed to write param '{}'".format(param_name))
                return False
        print("Successfully written {} = {}".format(param_name, value))
        return True

    def enter_at_mode(self):
        success = False
        retries = 5
        while True:
            print("Entering AT mode...")
            self._port.write(b"\r\n")
            self._port.write(self.AT_COMMAND["exit"].encode())
            time.sleep(1.5)
            self._port.write(b"+++")
            time.sleep(1.5)
            success = self.read_until_ok()
            if success:
                self._port.flushInput()
                return success
            print("Could not enter AT mode. Keep trying...")
            time.sleep(1.5)
            retries -= 1
            print("{} retriues left".format(retries))
            if retries == 0:
                self._port.flushInput()
                return False

    def read_parameters_from_eeprom(self):
        pattern = self._get_param_pattern()
        if pattern is None:
            print("Exiting...")
            exit(1)

        parameters = {}
        self._port.write(self.AT_COMMAND["show_eeprom"].encode())
        while True:
            answer = self._port.readline().decode("utf-8")
            answer = answer.lstrip().rstrip()
            if not answer:
                break
            match = pattern.search(answer)
            if match is not None:
                parameters[match.group(1).lower()] = int(match.group(2))
        self._current_params = parameters
        return parameters

    def _read_param_format(self):
        self._port.write("ATS0?\r\n".encode())
        param_format = None
        while True:
            answer = self._port.readline().decode("utf-8")
            if not answer:
                break
            try:
                param_format = int(answer)
            except:
                pass
            else:
                break
        return param_format

    def _get_param_pattern(self):
        param_format = self._read_param_format()
        pattern = None
        if param_format == 26:
            pattern = re.compile(r"[0-9]+:([a-zA-z_]*)=([0-9]*)")
        else:
            print("Paramter format '{}' not supported. Please implement it!".
                  format(param_format))
        return pattern

    def get_version(self):
        self._port.write(self.AT_COMMAND["show_version"].encode())
        version = ""
        while True:
            bytes = self._port.readline()
            print(bytes)
            answer = bytes.decode("utf-8")
            if not answer:
                break
            if answer == self.AT_COMMAND["show_version"]:
                continue
            version = answer
            break
        return version

    def print_parameters_table(self, param_current=None, param_desired=None):
        table = []
        if param_current is None:
            param_current = self._current_params
        if param_desired is None:
            param_desired = self._desired_params
        for key in param_current:
            row = []
            row.append(key)
            try:
                row.append(self.AT_EEPROM[key])
            except KeyError:
                row.append("")
            row.append(param_current[key])
            try:
                row.append(param_desired[key])
            except TypeError:
                row.append("")
            except KeyError:
                row.append("")
            table.append(row)
        print(tabulate(table,
                       headers=["Param", "ATSn", "Current", "Desired"],
                       tablefmt="pretty"))


def main():
    c = BaseConfigurator()
    if not c.enter_at_mode():
        print("Failed to enter AT mode.")
        exit(1)
    print(c.get_version())

    c.read_parameters_from_eeprom()
    c.set_desired_params(c.EEPROM_PARAMETERS)
    c.write_params()
    c.read_parameters_from_eeprom()
    c.print_parameters_table()
    c.write_eeprom()
    c.reboot()
    time.sleep(2.0)
    if not c.enter_at_mode():
        print("Failed to enter AT mode.")
        exit(1)
    c.read_parameters_from_eeprom()
    c.print_parameters_table()


if __name__ == "__main__":
    main()
