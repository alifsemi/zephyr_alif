.. _alif_e7_ak:

Alif E7 Application Kit
#######################

Overview
********

The Alif E7 Application Kit is a development board featuring the Alif Semiconductor
Ensemble E7 SoC with dual Arm Cortex-M55 processors and 2× Ethos-U55 NPUs.

Hardware
********

- Alif Ensemble E7 SoC
- Dual Arm Cortex-M55 processors (High-Performance + High-Efficiency)
- 2× Arm Ethos-U55 NPUs (256 MAC/cycle + 128 MAC/cycle)
- Integrated MRAM (on-chip non-volatile memory)
- USB connectivity
- Multiple GPIO, I2C, SPI, UART interfaces

.. note::
   The E7 supports both RTSS-HE (High-Efficiency) and RTSS-HP (High-Performance)
   Cortex-M55 core targets.

Serial Port
***********

The default console UART varies by core:

- RTSS-HE (High-Efficiency): UART2 at 115200 baud, 8N1
- RTSS-HP (High-Performance): UART4 at 115200 baud, 8N1

Supported Features
==================

The alif_e7_ak board configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| SYSTICK   | on-chip    | systick                             |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port                         |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+

Programming and Debugging
*************************

.. note::
   Alif Security Toolkit is required for ``west flash`` support.

Flashing
========

The Alif E7 AppKit supports flashing via JTAG/SWD using external debug probes.

Build and flash applications as usual:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e7_ak/ae722f80f55d5xx/rtss_he
   :goals: build flash

For the High-Performance core:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e7_ak/ae722f80f55d5xx/rtss_hp
   :goals: build flash

Debugging
=========

Debug using J-Link debugger via JTAG/SWD interface.

References
**********

- `Alif Semiconductor Website <https://alifsemi.com/>`_
