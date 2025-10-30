.. _snippet-alif-dk:

Alif support for Input (Touch sensor)
#####################################

Overview
********

This snippet enables input_dump test app for alif boards

Building and Running
********************

Example command to builf for E7 board (RTSS_HE devkit):

.. code-block::
   west build -p auto -b alif_e7_dk/ae722f80f55d5xx/rtss_he \
   samples/subsys/input/input_dump/ -S alif-dk

This application can be found under :
zephy_file: `samples/subsys/input/input_dump` in the zephyr tree.
