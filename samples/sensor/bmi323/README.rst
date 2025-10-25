.. _bmi323:

BMI323: Bosch Motion Tracking Device
##########################################

Description
***********

This sample application periodically measures the sensor
temperature, acceleration and angular velocity
displaying the values on the console along with a timestamp since
startup.

Wiring
*******

This sample uses an external breakout for the sensor.  A devicetree
overlay must be provided to identify the I3C  bus and GPIO (if required) used to
control the sensor.

Building and Running
********************

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

Integrated I3C controller
=========================

For Alif boards:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/bmi323
   :boards:
        alif_e7_dk_rtss_he
        alif_e7_dk_rtss_hp
        alif_e4_dk_rtss_he
        alif_e4_dk_rtss_hp
        alif_e8_dk_rtss_he
        alif_e8_dk_rtss_hp
        alif_e3_dk_rtss_hp
        alif_e3_dk_rtss_he
        alif_e1c_dk_rtss_he
        alif_b1_dk_rtss_he
   :goals: build flash

Example command to builf for E7 board (RTSS_HE devkit):

.. code-block::
   west build -p auto -b alif_e7_dk/ae722f80f55d5xx/rtss_he \
   samples/sensor/bmi323 -S alif-dk

Sample Output
=============

.. code-block:: console

<repeats endlessly>
