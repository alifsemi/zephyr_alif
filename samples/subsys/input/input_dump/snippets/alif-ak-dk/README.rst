.. _snippet-alif-ak-dk:

Alif support for Input (GT911 Touch sensor)
#####################################

Overview
********

This snippet enables input_dump test app for alif boards.
The input system fetches the display co-ordinates
where touch happened on the screen. It dumps the same on the console.

Building and Running
********************

This project outputs input co-ordinates data to the console.
It requires a GT911 Touch sensor. It should work with any platform
featuring a I2C peripheral interface. It does not work on QEMU.
In this example below the :ref:`alif_e7_dk/ae722f80f55d5xx/rtss_he` board is used.

After providing a devicetree overlay that specifies the sensor location,
build this sample app using:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/input/input_dump
   :board: alif_e7_dk/ae722f80f55d5xx/rtss_he
   :goals: build
   :gen-args: -S alif-ak-dk

Sample Output
=============

.. code-block:: console
        Input sample started
        I: input event: dev=gt911@5d             type= 3 code=  0 value=210
        I: input event: dev=gt911@5d             type= 3 code=  1 value=304
        I: input event: dev=gt911@5d         SYN type= 1 code=330 value=1
        I: input event: dev=gt911@5d             type= 3 code=  0 value=210
        I: input event: dev=gt911@5d             type= 3 code=  1 value=304
        I: input event: dev=gt911@5d         SYN type= 1 code=330 value=0
        I: input event: dev=gt911@5d             type= 3 code=  0 value=326
        I: input event: dev=gt911@5d             type= 3 code=  1 value=377
        I: input event: dev=gt911@5d         SYN type= 1 code=330 value=1
        I: input event: dev=gt911@5d             type= 3 code=  0 value=326
        I: input event: dev=gt911@5d             type= 3 code=  1 value=377
        I: input event: dev=gt911@5d         SYN type= 1 code=330 value=1
        I: input event: dev=gt911@5d             type= 3 code=  0 value=326
        I: input event: dev=gt911@5d             type= 3 code=  1 value=377
        I: input event: dev=gt911@5d         SYN type= 1 code=330 value=1

        <repeats endlessly>
