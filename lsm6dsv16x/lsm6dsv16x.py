# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# Modified by for Dingo V2
# SPDX-License-Identifier: MIT
"""
This module provides the `adafruit_lsm6ds.LSM6DSV16X` subclass of LSM6DS sensors
==============================================================================
"""
import sys
import os
from adafruit_register.i2c_struct import ROUnaryStruct, Struct
from adafruit_register.i2c_bits import RWBits, ROBits
from adafruit_register.i2c_bit import RWBit, ROBit
from micropython import const
from dataclasses import dataclass
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from __init__ import LSM6DS, CV, _LSM6DS_EMB_FUNC_INIT_A, LSM6DS_DEFAULT_ADDRESS, _LSM6DS_EMB_FUNC_EN_A, _LSM6DS_CTRL3_C

_LSM6DSV16X_SFLP_ODR = const(0x5E)
_LSM6DSV16X_EMB_FUNC_FIFO_EN_A = const(0x44)
_LSM6DSV16X_FIFO_CTRL3 = const(0x09)
_LSM6DSV16X_FIFO_CTRL4 = const(0x0A)
_LSM6DSV16X_FIFO_STATUS1 = const(0x1B)
_LSM6DSV16X_FIFO_DATA_OUT_X_L = const(0x79)
_LSM6DSV16X_FIFO_DATA_OUT_TAG = const(0x78)
_LSM6DSV16X_FIFO_CTRL1 = const(0x07)

LSM6DSV16X_CHIP_ID = const(0x70)


class SFLPRate(CV):
    """Options for ``accelerometer_data_rate`` and ``gyro_data_rate``"""


class FIFOMode(CV):
    """Options for ``accelerometer_data_rate`` and ``gyro_data_rate``"""


@dataclass
class FIFOStatus:
    samples: int
    wtm: bool
    ovr: bool
    full: bool
    bdr: bool
    ovr_latch: bool


SFLPRate.add_values(
    (
        ("RATE_15_HZ", 0, 15.0, None),
        ("RATE_30_HZ", 1, 30.0, None),
        ("RATE_60_HZ", 2, 60.0, None),
        ("RATE_120_HZ", 3, 120.0, None),
        ("RATE_240_HZ", 4, 240.0, None),
        ("RATE_480_HZ", 5, 480.0, None),
    )
)

FIFOMode.add_values(
    (
        ("LSM6DSV16X_BYPASS_MODE", 0, 0, None),
        ("LSM6DSV16X_FIFO_MODE", 1, 1, None),
        ("LSM6DSV16X_CONTINUOUS_WTM_TO_FULL_MODE", 2, 2, None),
        ("LSM6DSV16X_CONTINUOUS_TO_FIFO_MODE", 3, 3, None),
        ("LSM6DSV16X_BYPASS_TO_CONTINUOUS_MODE", 4, 4, None),
        ("LSM6DSV16X_CONTINUOUS_MODE", 6, 6, None),  # skip 5 (reserved)
        ("LSM6DSV16X_BYPASS_TO_FIFO_MODE", 7, 7, None),
    )
)

try:
    import typing  # pylint: disable=unused-import
    from busio import I2C
except ImportError:
    pass


class LSM6DSV16X(LSM6DS):  # pylint: disable=too-many-instance-attributes

    """Driver for the LSM6DSV16X 6-axis accelerometer and gyroscope.

    :param ~busio.I2C i2c_bus: The I2C bus the LSM6DSV16X is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x6A`


    **Quickstart: Importing and using the device**

        Here is an example of using the :class:`LSM6DSV16X` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from adafruit_lsm6ds.LSM6DSV16X import LSM6DSV16X

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()  # uses board.SCL and board.SDA
            sensor = LSM6DSV16X(i2c)

        Now you have access to the :attr:`acceleration` and :attr:`gyro`: attributes

        .. code-block:: python

            acc_x, acc_y, acc_z = sensor.acceleration
            gyro_x, gyro_z, gyro_z = sensor.gyro

    """

    CHIP_ID = LSM6DSV16X_CHIP_ID
    _sflp_data_rate = RWBits(3, _LSM6DSV16X_SFLP_ODR, 3)
    _sflp_game_vec_batch = RWBit(_LSM6DSV16X_EMB_FUNC_FIFO_EN_A, 1)
    _sflp_gbias_batch = RWBit(_LSM6DSV16X_EMB_FUNC_FIFO_EN_A, 5)
    _sflp_gravity_vec_batch = RWBit(_LSM6DSV16X_EMB_FUNC_FIFO_EN_A, 4)
    _acc_batch = RWBits(4, _LSM6DSV16X_FIFO_CTRL3, 0)
    _fifo_mode = RWBits(3, _LSM6DSV16X_FIFO_CTRL4, 0)

    _sflp_init = RWBit(_LSM6DS_EMB_FUNC_INIT_A, 1)
    _fifo_status1 = ROBits(16, _LSM6DSV16X_FIFO_STATUS1, 0, 2)

    _fifo_data_out_tag = ROBits(5, _LSM6DSV16X_FIFO_DATA_OUT_TAG, 3)
    _raw_sensor_fusion_data = Struct(_LSM6DSV16X_FIFO_DATA_OUT_X_L, "<hhh")
    _fifo_watermark = RWBits(8, _LSM6DSV16X_FIFO_CTRL1, 0)

    _sflp_en = RWBit(_LSM6DS_EMB_FUNC_EN_A, 1)

    SAMPLES_BITMASK = 0b0000000111111111
    WTM_BITMASK = 0b1000000000000000
    OVR_BITMASK = 0b0100000000000000
    FULL_BITMASK = 0b0010000000000000
    BDR_BITMASK = 0b0001000000000000
    OVR_LATCHED_BITMASK = 0b0000100000000000

    def __init__(
            self,
            i2c_bus: I2C,
            address: int = LSM6DS_DEFAULT_ADDRESS,
            ucf: str = None,
            sensor_fusion: bool = True
    ) -> None:
        super().__init__(i2c_bus, address, ucf)
        if sensor_fusion:
            self.enable_sflp()
        self._print_regs()

    def enable_sflp(self):
        self.sflp_data_rate = SFLPRate.RATE_480_HZ  # Set rate 0x5e
        self.fifo_mode = FIFOMode.LSM6DSV16X_CONTINUOUS_MODE
        self.sflp_en = True

    def _reinit_sflp(self):
        self.sflp_init = True

    @property
    def quaternion(self):
        return self._fifo_wait_and_get(0x13)

    @property
    def gbias(self):
        return self._fifo_wait_and_get(0x16)

    @property
    def gravity_vec(self):
        return self._fifo_wait_and_get(0x17)

    def _fifo_wait_and_get(self, tag):
        status = self.read_status()
        num_samples = status.samples
        if num_samples > 0:
            while True:
                # Grab data from FIFO, this also pops it.
                fifo_tag = self.fifo_data_out_tag
                data = self.raw_sensor_fusion_data
                if fifo_tag == tag:  # Check for quaternion tag
                    return data
        return None


    def read_status(self):
        raw_status = self.fifo_status1
        samples = raw_status & self.SAMPLES_BITMASK
        wtm = bool(raw_status & self.WTM_BITMASK)
        ovr = bool(raw_status & self.OVR_BITMASK)
        full = bool(raw_status & self.FULL_BITMASK)
        bdr = bool(raw_status & self.BDR_BITMASK)
        ovr_latch = bool(raw_status & self.OVR_LATCHED_BITMASK)
        return FIFOStatus(samples, wtm, ovr, full, bdr, ovr_latch)

    def _print_regs(self):
        print("Current registers:")
        print(f"data_rate: {bin(self.sflp_data_rate)}")
        print(f"batch: {bin(self.sflp_game_vec_batch)}")
        print(f"fifo mode: {bin(self.fifo_mode)}")
        print(f"sflp en: {bin(self.sflp_en)}")
        print(f"sflp init: {bin(self.sflp_init)}")
        print(f"fifo watermark: {bin(self.fifo_watermark)}")
        print(f"acc batch: {bin(self.acc_batch)}")
        print(f"gyro range: {bin(self.gyro_range)}")
        print(f"gyro rate: {bin(self.gyro_data_rate)}")
        print(f"acc range: {bin(self.accelerometer_range)}")
        print(f"acc rate: {bin(self.accelerometer_data_rate)}")
        print("=================================")

    @property
    def mem_bank(self) -> int:
        return self._mem_bank

    @mem_bank.setter
    def mem_bank(self, value: int) -> None:
        self._mem_bank = value

    @property
    def sflp_data_rate(self) -> int:
        self.mem_bank = 1
        ret = self._sflp_data_rate
        self.mem_bank = 0
        return ret

    @sflp_data_rate.setter
    def sflp_data_rate(self, value: int) -> None:
        self.mem_bank = 1
        self._sflp_data_rate = value
        self.mem_bank = 0

    @property
    def sflp_game_vec_batch(self) -> bool:
        self.mem_bank = 1
        ret = self._sflp_game_vec_batch
        self.mem_bank = 0
        return ret

    @sflp_game_vec_batch.setter
    def sflp_game_vec_batch(self, value: bool) -> None:
        self.mem_bank = 1
        self._sflp_game_vec_batch = value
        self.mem_bank = 0

    @property
    def fifo_mode(self) -> int:
        return self._fifo_mode

    @fifo_mode.setter
    def fifo_mode(self, value: int) -> None:
        self._fifo_mode = value

    @property
    def sflp_init(self) -> bool:
        self.mem_bank = 1
        ret = self._sflp_init
        self.mem_bank = 0
        return ret

    @sflp_init.setter
    def sflp_init(self, value: bool) -> None:
        self.mem_bank = 1
        self._sflp_init = value
        self.mem_bank = 0

    @property
    def fifo_status1(self) -> int:
        return self._fifo_status1

    @property
    def fifo_data_out_tag(self) -> int:
        return self._fifo_data_out_tag

    @property
    def raw_sensor_fusion_data(self) -> tuple:
        return self._raw_sensor_fusion_data

    @raw_sensor_fusion_data.setter
    def raw_sensor_fusion_data(self, value: tuple) -> None:
        self._raw_sensor_fusion_data = value

    @property
    def sflp_en(self) -> bool:
        self.mem_bank = 1
        ret = self._sflp_en
        self.mem_bank = 0
        return ret

    @sflp_en.setter
    def sflp_en(self, value: bool) -> None:
        self.mem_bank = 1
        self._sflp_en = value
        self.mem_bank = 0

    @property
    def fifo_watermark(self) -> int:
        return self._fifo_watermark

    @fifo_watermark.setter
    def fifo_watermark(self, value: int) -> None:
        self._fifo_watermark = value

    @property
    def acc_batch(self) -> int:
        return self._acc_batch

    @acc_batch.setter
    def acc_batch(self, value: int) -> None:
        self._acc_batch = value

    @property
    def sflp_g_bias_batch(self) -> bool:
        return self._sflp_gbias_batch

    @sflp_g_bias_batch.setter
    def sflp_g_bias_batch(self, value: bool) -> None:
        self._sflp_gbias_batch = value

    @property
    def sflp_gravity_vec_batch(self) -> bool:
        return self._sflp_gravity_vec_batch

    @sflp_gravity_vec_batch.setter
    def sflp_gravity_vec_batch(self, value: bool) -> None:
        self._sflp_gravity_vec_batch = value




