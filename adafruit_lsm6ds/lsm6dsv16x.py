# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# Modified by Cian Rodriguez for Dingo V2
# SPDX-License-Identifier: MIT
"""
This module provides the `adafruit_lsm6ds.LSM6DSV16X` subclass of LSM6DS sensors
==============================================================================
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from __init__ import LSM6DS, LSM6DS_DEFAULT_ADDRESS, LSM6DSV16X_CHIP_ID

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

    def __init__(
        self, 
        i2c_bus: I2C, 
        address: int = LSM6DS_DEFAULT_ADDRESS, 
        ucf: str = None,
        sensor_fusion: bool = True
    ) -> None:
        
        super().__init__(i2c_bus, address, ucf)
        self._i3c_disable = True

        if sensor_fusion:
            _emb_func_en_a = (i2c_bus, 0x2)
            _emb_func_en_b = (i2c_bus, 0x0)
            self._set_embedded_functions(True, (_emb_func_en_a, _emb_func_en_b))

    @property
    def rot_vector(self):
        print(self._fifo_status) 

