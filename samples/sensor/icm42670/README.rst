.. _icm42670:

ICM42670: Invensense Motion Tracking Device
##########################################

Description
***********

This sample application periodically measures the sensor
temperature, acceleration and angular velocity
displaying the values on the console along with a timestamp since
startup.

Wiring
*******

No wiring needed as this is an on-board sensor. A devicetree
overlay must be provided to identify the I3C  bus and GPIO (if required) used to
control the sensor.

Building and Running
********************

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

For Alif boards:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/icm42670
   :boards:
        alif_e7_dk_rtss_hp
        alif_e7_dk_rtss_he
        alif_e3_dk_rtss_hp
        alif_e3_dk_rtss_he
        alif_e1c_dk_rtss_he
        alif_b1_dk_rtss_he
   :goals: build flash

Sample Output
=============

.. code-block:: console

    [00:00:00.000,000] <dbg> i3c_dw: dw_i3c_attach_device: i3c@49034000: Attaching icm42670@680000023500000000
    [00:00:00.000,000] <dbg> i3c_dw: set_controller_info: i3c@49034000: 0x08 DA selected for controller
    [00:00:00.000,000] <dbg> i3c_dw: dw_i3c_reattach_device: Reattaching icm42670@680000023500000000
    [00:00:00.004,000] <dbg> ICM42670: icm42670_sensor_init: device id: 0x67
    *** Booting Zephyr OS build ZAS-v1.0.0-rc1-41-g97369682bd80 ***
    [00:00:00.105,000] <inf> ICM42670: Configured for triggered sampling.

    [00:00:00.169,000] <inf> ICM42670: temp  25.6875 Cel     accel -0.095769 -0.057461  9.964667 m/s/s       gyro  -0.002128 -0.007449 -0.006385 rad/s
    [00:00:00.233,000] <inf> ICM42670: temp  25.875 Cel      accel -0.105345 -0.043096  9.964667 m/s/s       gyro  -0.003192 -0.007449 -0.005321 rad/s
    [00:00:00.297,000] <inf> ICM42670: temp  25.875 Cel      accel -0.100557 -0.043096  9.955090 m/s/s       gyro  -0.003192 -0.006385 -0.005321 rad/s
    [00:00:00.361,000] <inf> ICM42670: temp  25.875 Cel      accel -0.076615 -0.057461  9.959878 m/s/s       gyro  -0.002128 -0.009578 -0.006385 rad/s
    [00:00:00.425,000] <inf> ICM42670: temp  25.9375 Cel     accel -0.095769 -0.043096  9.964667 m/s/s       gyro  -0.002128 -0.007449 -0.006385 rad/s
    [00:00:00.490,000] <inf> ICM42670: temp  25.9375 Cel     accel -0.105345 -0.071827  9.950302 m/s/s       gyro  -0.004256 -0.006385 -0.005321 rad/s
    [00:00:00.554,000] <inf> ICM42670: temp  25.8438 Cel     accel -0.110134 -0.057461  9.969455 m/s/s       gyro  -0.002128 -0.007449 -0.007449 rad/s
    [00:00:00.618,000] <inf> ICM42670: temp  25.9375 Cel     accel -0.081403 -0.052673  9.969455 m/s/s       gyro  -0.002128 -0.009578 -0.006385 rad/s

<repeats endlessly>
