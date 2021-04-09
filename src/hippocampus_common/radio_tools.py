from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals

import serial
import time
import re


class Firmware(object):
    def __init__(self, path):
        self.ranges = dict()
        self.upperaddress = 0x0000
        self.banking_detected = False
        self.sanity_check = dict()


class FirmwareUploader(object):
    NOP = chr(0x00)
    OK = chr(0x10)
    FAILED = chr(0x11)
    INSYNC = chr(0x12)
    EOC = chr(0x20)
    GET_SYNC = chr(0x21)
    GET_DEVICE = chr(0x22)
    CHIP_ERASE = chr(0x23)
    LOAD_ADDRESS = chr(0x24)
    PROG_FLASH = chr(0x25)
    READ_FLASH = chr(0x26)
    PROG_MULTI = chr(0x27)
    READ_MULTI = chr(0x28)
    PARAM_ERASE = chr(0x29)
    REBOOT = chr(0x30)

    PROG_MULTI_MAX = 32  # 64 causes serial hangs with some USB-serial adapters
    READ_MULTI_MAX = 255
    BANK_PROGRAMING = -1

    def __init__(self, port=None, at_baud=57600, bootloader_baud=115200):
        self.port = serial.Serial(port=port,
                                  baudrate=bootloader_baud,
                                  timeout=3)
        self.at_baud = at_baud
        self.bootloader_baud = bootloader_baud

    def _read(self):
        c = self.port.read().decode("utf-8")
        if len(c) < 1:
            raise RuntimeError("Read timeout.")
        return c

    def send(self, string):
        self.port.write(string.encode())

    def _get_sync(self):
        c = self._read()
        if c != self.INSYNC:
            raise RuntimeError("Sync failed. Expected INSYNC=0x{:02X} but got "
                               "0x{:02X}.".format(ord(self.INSYNC), ord(c)))
        c = self._read()
        if c != self.OK:
            raise RuntimeError("Sync failed. Expected OK=0x{:02X} but got "
                               "0x{:02X}.".format(ord(self.OK), ord(c)))

    def _sync(self):
        self.send(self.PROG_MULTI_MAX)
        self.port.reset_input_buffer()
        self.send(self.GET_SYNC + self.EOC)
        self._get_sync()

    def _erase(self, erase_params=False):
        self.send(self.CHIP_ERASE + self.EOC)
        self._get_sync()
        if erase_params:
            self.send(self.PARAM_ERASE + self.EOC)
            self._get_sync()

    def _set_address(self, address, banking):
        if banking:
            if self.BANK_PROGRAMING != (address >> 16):
                self.BANK_PROGRAMING = address >> 16
                if self.BANK_PROGRAMING == 0:
                    print("HOME")
                else:
                    print("BANK", self.BANK_PROGRAMING)
            self.send(self.LOAD_ADDRESS + chr(address & 0xFF) +
                      chr((address >> 8) & 0xFF) + chr((address >> 16) & 0xFF) +
                      self.EOC)
        else:
            self.send(self.LOAD_ADDRESS + chr(address & 0xFF) +
                      chr(address >> 8) + self.EOC)
        self._get_sync()

    def _program_single(self, data):
        self.send(self.PROG_FLASH + chr(data) + self.EOC)
        self.__get_sync()

    def _program_multi(self, data):
        sync_count = 0
        while len(data):
            n = min(len(data), self.PROG_MULTI_MAX)
            block = data[:n]
            block = data[:n]
            data = data[n:]
            self.send(self.PROG_MULTI + chr(n))
            self.send(block)
            self.send(self.EOC)
            sync_count += 1
        for _ in range(sync_count):
            self._get_sync()

    def _verify_byte(self, data):
        self.send(self.READ_FLASH + self.EOC)
        if self._read() != chr(data):
            return False
        self._get_sync()
        return True

    def _verify_multi(self, data):
        self.send(self.READ_MULTI + chr(len(data)) + self.EOC)
        for i in data:
            if self._read() != chr(i):
                return False
        self._get_sync()
        return True

    def _reboot(self):
        self.send(self.REBOOT)

    def _split(self, seq, length):
        return [seq[i:i + length] for i in range(0, len(seq), length)]

    def total_size(self, code):
        total = 0
        for address in code.keys():
            total += len(code[address])
        return total

    def _program(self, firmware):
        code = firmware.code()
        count = 0
        total = self.total_size(code)
        for address in sorted(code.keys()):
            self._set_address(address, firmware.banking_deteted)
            groups = self._split(code[address], self.PROG_MULTI_MAX)
            for bytes in groups:
                self._program_multi(bytes)
                count += len(bytes)

    def _verify(self, firmware):
        code = firmware.code()
        count = 0
        total = self.total_size(code)
        for address in sorted(code.keys()):
            self._set_address(address, firmware.banking_deteted)
            groups = self._split(code[address], self.READ_MULTI_MAX)
            for bytes in groups:
                if not self._verify_multi(bytes):
                    raise RuntimeError(
                        "Verification failed at 0x{:02X}".format(address))
                count += len(bytes)

    def expect(self, pattern, timeout):
        pattern = re.compile(pattern)
        start = time.time()
        s = ""
        while time.time() < start + timeout:
            b = self.port.read.decode("utf-8")
            if b:
                if pattern.search(s) is not None:
                    return True
            else:
                time.sleep(0.01)
        return False

    def sync_at(self):
        if self.at_baud != self.port.baudrate:
            self.port.baudrate = self.at_baud
        self.send("\r\n")
        time.sleep(1.0)
        self.send("+++")
        self.expect("OK", timeout=2.0)
        for i in range(5):
            self.send("\r\nATI\r\n")
            if not self.expect("SiK .* on", timeout=0.5):
                print("Failed to get SiK banner")
                continue
            else:
                return True
        return False

    def autosync(self):
        if self.sync_at():
            self.send("\r\n")
            time.sleep(0.2)
            self.port.reset_input_buffer()
            self.send("AT&UPDATE\r\n")
            time.sleep(0.7)
            self.port.reset_input_buffer()
            self.port.baudrate = self.bootloader_baud
            return True
        self.port.baudrate = self.bootloader_baud
        return False

    def check_bootloader(self):
        for i in range(3):
            try:
                if self._sync():
                    return True
                self.autosync()
            except RuntimeError:
                self.autosync()
        return False

    def identify(self):
        self._send(self.GET_DEVICE + self.EOC)
        board_id = ord(self._read()[0])
        board_freq = ord(self._read()[0])
        self._get_sync()
        return board_id, board_freq

    def upload(self, firmware, erase_params=False):
        if not self.check_bootloader():
            raise RuntimeError("Failed to contact bootloader")
        board_id, board_freq = self.identify()
        if firmware.banking_detected and not (board_id & 0x80):
            raise RuntimeError("This firmware requires a CPU with banking.")
        if (board_id & 0x80):
            firmware.banking_detected = True
        self._erase(erase_params=erase_params)
        self._program(firmware)
        self._verify(firmware)
        self._reboot()


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
        netid=dict(register=3,
                   readonly=False,
                   options=list(range(0, 26)),
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
