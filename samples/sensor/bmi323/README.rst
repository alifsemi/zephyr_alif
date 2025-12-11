.. _bmi323:

BMI323: Bosch Motion Tracking Device
##########################################

Description
***********

This sample application periodically measures the sensor
temperature, acceleration and angular velocity
displaying the values on the console.

Wiring
*******

This sample uses an external breakout for the sensor. A devicetree
overlay must be provided to identify the I3C bus and GPIO (if required)
used to control the sensor.

Building and Running
********************

This project outputs sensor data to the console. It requires a BMI323 sensor.
It should work with any platform featuring a I2C peripheral interface.
It does not work on QEMU.
In this example below the :ref:`alif_e7_dk/ae722f80f55d5xx/rtss_he` board is used.

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/bmi323
   :board: alif_e7_dk/ae722f80f55d5xx/rtss_he
   :goals: build
   :gen-args: -S alif-dk-ak

Sample Output
=============

.. code-block:: console
        Device 0xb284 name is bmi323@69000003b810431000
        Accel AX: -0.002989; AY: -0.445544; AZ: -0.870592 g      Gyro GX: -0.274662; GY: -0.213626; GZ: 0.045777 deg/s
        Accel AX: -0.002074; AY: -0.446093; AZ: -0.869860 g      Gyro GX: -0.274662; GY: -0.259403; GZ: 0.106813 deg/s
        Accel AX: -0.001891; AY: -0.445422; AZ: -0.868884 g      Gyro GX: -0.305180; GY: -0.198367; GZ: 0.122072 deg/s
        Accel AX: -0.002257; AY: -0.446764; AZ: -0.871141 g      Gyro GX: -0.320439; GY: -0.213626; GZ: 0.030518 deg/s
        Accel AX: -0.000671; AY: -0.446337; AZ: -0.870287 g      Gyro GX: -0.305180; GY: -0.183108; GZ: 0.061036 deg/s
        Accel AX: 0.000366; AY: -0.446337; AZ: -0.870409 g       Gyro GX: -0.274662; GY: -0.167849; GZ: 0.076295 deg/s
        Accel AX: -0.000915; AY: -0.446276; AZ: -0.870165 g      Gyro GX: -0.259403; GY: -0.228885; GZ: 0.061036 deg/s
        Accel AX: -0.001586; AY: -0.446520; AZ: -0.870287 g      Gyro GX: -0.244144; GY: -0.228885; GZ: 0.091554 deg/s
        Accel AX: -0.001098; AY: -0.447496; AZ: -0.869372 g      Gyro GX: -0.305180; GY: -0.274662; GZ: 0.091554 deg/s
        Accel AX: -0.002440; AY: -0.447008; AZ: -0.869555 g      Gyro GX: -0.305180; GY: -0.183108; GZ: 0.106813 deg/s
        Accel AX: -0.000488; AY: -0.446276; AZ: -0.870226 g      Gyro GX: -0.320439; GY: -0.213626; GZ: 0.091554 deg/s

        <repeats endlessly>
