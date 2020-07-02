import time
import smbus2

RESET_CMD = 0x1E
CONV_D1_OSR256 = 0x40
CONV_D1_OSR512 = 0x42
CONV_D1_OSR1024 = 0x44
CONV_D1_OSR2048 = 0x46
CONV_D1_OSR4096 = 0x48
CONV_D2_OSR256 = 0x50
CONV_D2_OSR512 = 0x52
CONV_D2_OSR1024 = 0x54
CONV_D2_OSR2048 = 0x56
CONV_D2_OSR4096 = 0x58
ADC_READ = 0x00
PROM_READ_BASE = 0xA0


class MS5803(object):
    """Abstraction of the MS5803 pressure sensor.
    """
    def __init__(self, address=0x76):
        """
        Args:
            address (hexadecimal, optional): I2C-address of the sensor. Defaults to 0x76.
        """
        self.bus = smbus2.SMBus(1)
        time.sleep(1.0)
        self.address = address
        self.calib_data = {}
        self._d1_resolutions = {
            256: CONV_D1_OSR256,
            512: CONV_D1_OSR512,
            1024: CONV_D1_OSR1024,
            2048: CONV_D1_OSR2048,
            4096: CONV_D1_OSR4096,
        }
        self._d2_resolutions = {
            256: CONV_D2_OSR256,
            512: CONV_D2_OSR512,
            1024: CONV_D2_OSR1024,
            2048: CONV_D2_OSR2048,
            4096: CONV_D2_OSR4096,
        }
        self.d1_res = self._d1_resolutions[4096]
        self.d2_res = self._d2_resolutions[4096]

        self._read_calib_data()

    def reset(self):
        """Resets the sensor. Not needed during normal operation.
        """
        self.bus.write_byte(self.address, RESET_CMD)

    def _read_d1(self):
        """Starts a pressure conversion and reads the raw ADC value.

        Returns:
            (int): Raw sensor value.
        """
        self.bus.write_byte(self.address, self.d1_res)
        time.sleep(0.01)
        raw_data = self.bus.read_i2c_block_data(self.address, ADC_READ, 3)
        return (raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]

    def _read_d2(self):
        """Starts a temperature conversion and reads the raw ADC value.

        Returns:
            (int): Raw sensor value.
        """
        self.bus.write_byte(self.address, self.d2_res)
        time.sleep(0.01)
        raw_data = self.bus.read_i2c_block_data(self.address, ADC_READ, 3)
        return (raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]

    def _read_prom(self, address):
        """Reads the PROM content at the specified address.

        Args:
            address (int): Address of the PROM

        Returns:
            (int): Raw content of the PROM content at the specified address.
        """
        address = (address & 0x07) << 1
        raw_data = self.bus.read_i2c_block_data(self.address,
                                                PROM_READ_BASE | address, 2)
        return (raw_data[0] << 8) | raw_data[1]

    def _calculate_temperature(self, D2):
        """Calculates the temperature based on raw sensor reading and calibration data.

        Args:
            D2 (int): Raw temperature read from ADC.

        Returns:
            (tuple): Tuple of compensated temperature in hundredth °C and the difference from the calibration reference temperature needed for calculating the temperature compensated pressure.
        """
        dT = D2 - self.calib_data["c5"] * 2**8
        return (2000 + dT * self.calib_data["c6"] / (2**23)), dT

    def _calculate_pressure(self, D1, T, dT):
        """Calculates the temperature compensated pressure.

        Args:
            D1 (int): Raw pressure value from ADC.
            T (float): Compensated temperature.
            dT (float): Difference to reference temeprature.

        Returns:
            (float): Temperature compensated pressure [Pa].
        """
        OFF = self.calib_data["c2"] * 2**16 + (self.calib_data["c4"] *
                                               dT) / (2**7)
        SENS = self.calib_data["c1"] * 2**15 + (self.calib_data["c3"] *
                                                dT) / (2**8)
        return (D1 * SENS / (2**21) - OFF) / (2**15)

    def _read_calib_data(self):
        """Reads the calibration data from the PROM.
        """
        self.calib_data["c1"] = self._read_prom(1)
        self.calib_data["c2"] = self._read_prom(2)
        self.calib_data["c3"] = self._read_prom(3)
        self.calib_data["c4"] = self._read_prom(4)
        self.calib_data["c5"] = self._read_prom(5)
        self.calib_data["c6"] = self._read_prom(6)

    def get_measurement(self):
        """Get current pressure and temperature. Performs a full reading and compensation cycle.

        Returns:
            (tuple): Tuple of pressure [Pa] and temperature [°C]
        """
        D1 = self._read_d1()
        D2 = self._read_d2()
        T, dT = self._calculate_temperature(D2)
        P = self._calculate_pressure(D1, T, dT)
        T = T * 0.01
        return P, T
