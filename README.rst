========================
CircuitPython LSM6DSV16X
========================

This fork was created to implement the LSM6DSV16X 6DoF IMU from STM. It implements acceleration and gyro data, as well as sensor fusion between the two. PRs to implement the remaining features are welcome. 

**Backwards compatability with other LSM6DS family sensors implemented in the original Adafruit library has not been tested, and is likely broken.**

The LSM6DSV16X is not currently an Adafruit product. 

Installation
===========
Using :code:`pip3 install adafruit-circuitpython-lsm6ds` will install the original Adafruit library, and which currently does *not* support the LSM6DSV16X. You must clone this repo as a submodule into your project.

.. code-block::

    git submodule add https://github.com/SpaceBarrr/Adafruit_CircuitPython_LSM6DS
    git submodule update --init

Dependencies are the same as the original Adafruit library. This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_


Usage
=====
By default, sensor fusion is enabled. You can access the fused data through the :code:`quaternion` property. However, you must set the values to be batched and then re-init.

.. code-block:: python3

    import time
    import board
    
    from lsm6dsv16x import LSM6DSV16X
    
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor: LSM6DSV16X = LSM6DSV16X(i2c, sensor_fusion=True)
    sensor.fifo_mode = FIFOMode.CONTINUOUS_WTM_TO_FULL_MODE
    sensor.fifo_watermark = 1
    sensor.sflp_game_vec_batch = True
    sensor.sflp_gravity_vec_batch = False
    sensor.sflp_g_bias_batch = False
    sensor.sflp_init = True
    
    while True:
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyro))
        print(f"Temp: {sensor.temperature}")
        print(f"Quaternion: {sensor.quaternion}")
        time.sleep(0.5)

Documentation
=============
API documentation for the original library can be found on `Read the Docs <https://docs.circuitpython.org/projects/lsm6dsox/en/latest/>`_.

Additional features were implemented based off the `LSM6DSV16X datasheet <https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf>`_ and the `example C code repository <https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_sensor_fusion.c>`_.
