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
from enum import IntEnum
import time
import numpy as np

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
        ("BYPASS_MODE", 0, 0, None),
        ("FIFO_MODE", 1, 1, None),
        ("CONTINUOUS_WTM_TO_FULL_MODE", 2, 2, None),
        ("CONTINUOUS_TO_FIFO_MODE", 3, 3, None),
        ("BYPASS_TO_CONTINUOUS_MODE", 4, 4, None),
        ("CONTINUOUS_MODE", 6, 6, None),  # skip 5 (reserved)
        ("BYPASS_TO_FIFO_MODE", 7, 7, None),
    )
)

try:
    import typing  # pylint: disable=unused-import
    from busio import I2C
except ImportError:
    pass


class LSM6DSV16XDataTags(IntEnum):
    FIFO_empty = 0x00
    Gyroscope_NC = 0x01
    Accelerometer_NC = 0x02
    Temperature = 0x03
    Timestamp = 0x04
    CFG_Change = 0x05
    Accelerometer_NC_T_2 = 0x06
    Accelerometer_NC_T_1 = 0x07
    Accelerometer_2xC = 0x08
    Accelerometer_3xC = 0x09
    Gyroscope_NC_T_2 = 0x0A
    Gyroscope_NC_T_1 = 0x0B
    Gyroscope_2xC = 0x0C
    Gyroscope_3xC = 0x0D
    Sensor_hub_slave_0 = 0x0E
    Sensor_hub_slave_1 = 0x0F
    Sensor_hub_slave_2 = 0x10
    Sensor_hub_slave_3 = 0x11
    Step_counter = 0x12
    SFLP_game_rotation_vector = 0x13
    SFLP_gyroscope_bias = 0x16
    SFLP_gravity_vector = 0x17
    Sensor_hub_nack = 0x19
    MLC_result = 0x1A
    MLC_filter = 0x1B
    MLC_feature = 0x1C
    Accelerometer_dualC = 0x1D
    Enhanced_EIS_gyroscope = 0x1E

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
    _raw_sensor_fusion_data = Struct(_LSM6DSV16X_FIFO_DATA_OUT_X_L, "<eee")
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

        self._fifo_data_tags = []

    def enable_sflp(self):
        self.sflp_data_rate = SFLPRate.RATE_480_HZ  # Set rate 0x5e
        self.fifo_mode = FIFOMode.CONTINUOUS_MODE
        self.sflp_en = True

    @property
    def quaternion(self):
        return self._process_quaternion(self._fifo_wait_and_get(0x13))

    @property
    def gbias(self):
        return self._fifo_wait_and_get(0x16)

    @property
    def gravity_vec(self):
        return self._fifo_wait_and_get(0x17)

    @property
    def fifo_batch_data(self):
        """
        Gets all the data enabled in batching, this will wait until at least one of each value has been obtained.
        If you just want one value, use dedicated parameter instead.
        :return: Dictionary
        """
        status = self.fifo_status
        num_samples = status.samples
        if num_samples > 0:
            ret = {}
            while len(ret.keys()) < len(self._fifo_data_tags):
                # Grab data from FIFO, this also pops it.
                fifo_tag = self.fifo_data_out_tag
                data = self.raw_sensor_fusion_data
                if fifo_tag in self._fifo_data_tags:
                    if fifo_tag == LSM6DSV16XDataTags.SFLP_game_rotation_vector:
                        data = self._process_quaternion(data)
                    ret[LSM6DSV16XDataTags(fifo_tag).name] = data
            return ret
        return None

    def _fifo_wait_and_get(self, tag, timeout=1):
        """
        Wait and get data from fifo based of tag. This will erase all data in the FIFO up until the data required.
        :param tag: FIFO data tag as specified in LSM6DSV16XDataTags enum.
        :return: data in FIFO, None if samples is 0 or timeout hit.
        """
        status = self.fifo_status
        num_samples = status.samples
        time_start = time.time()
        if num_samples > 0:
            while time.time() - time_start < timeout:
                # Grab data from FIFO, this also pops it.
                fifo_tag = self.fifo_data_out_tag
                data = self.raw_sensor_fusion_data
                if fifo_tag == tag:  # Check for quaternion tag
                    return data
        return None

    @property
    def fifo_status(self):
        """
        Processes FIFO status message.
        :return: FIFOStatus dataclass
        """
        raw_status = self.raw_fifo_status
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
        self._adjust_data_tag(LSM6DSV16XDataTags.SFLP_game_rotation_vector, value)

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
    def raw_fifo_status(self) -> int:
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
        self.mem_bank = 1
        self._sflp_gbias_batch = value
        self.mem_bank = 0
        self._adjust_data_tag(LSM6DSV16XDataTags.SFLP_gyroscope_bias, value)

    @property
    def sflp_gravity_vec_batch(self) -> bool:
        return self._sflp_gravity_vec_batch

    @sflp_gravity_vec_batch.setter
    def sflp_gravity_vec_batch(self, value: bool) -> None:
        self.mem_bank = 1
        self._sflp_gravity_vec_batch = value
        self.mem_bank = 0
        self._adjust_data_tag(LSM6DSV16XDataTags.SFLP_gravity_vector, value)

    def _adjust_data_tag(self, tag, value):
        self._fifo_data_tags.append(tag) if value else self._fifo_data_tags.remove(tag)

    @staticmethod
    def _process_quaternion(quaternion):
        quaternion = np.array(quaternion)
        sumsq = np.sum(quaternion**2)
        
        if sumsq > 1:
            n = np.sqrt(sumsq)
            quaternion[0] /= n
            quaternion[1] /= n
            quaternion[2] /= n
            sumsq = 1
        quaternion = np.insert(quaternion, 0, np.sqrt(1 - sumsq))
        # print(quaternion)
        return quaternion
