.. _alif_e8_ak:

Alif E8 Application Kit
#######################

Overview
********

The Alif E8 Application Kit is a development board featuring the Alif Semiconductor
Ensemble E8 SoC with dual Arm Cortex-M55 processors, Ethos-U55 NPU, and Ethos-U85 NPU.

Hardware
********

- Alif Ensemble E8 SoC
- Dual Arm Cortex-M55 processors (High-Performance + High-Efficiency)
- 1× Arm Ethos-U85 NPU for Generative AI (256 MAC/cycle, up to 204 GOPS)
- 2× Arm Ethos-U55 NPUs (256 MAC/cycle + 128 MAC/cycle)
- Integrated MRAM (on-chip non-volatile memory)
- USB connectivity
- Multiple GPIO, I2C, SPI, UART interfaces

.. note::
   The E8 supports both RTSS-HE (High-Efficiency) and RTSS-HP (High-Performance)
   Cortex-M55 core targets.

Serial Port
***********

The default console UART varies by core:

- RTSS-HE (High-Efficiency): UART2 at 115200 baud, 8N1
- RTSS-HP (High-Performance): UART4 at 115200 baud, 8N1

Supported Features
==================

The alif_e8_ak board configuration supports the following hardware features:

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

The Alif E8 AppKit supports flashing via JTAG/SWD using external debug probes.

Build and flash applications as usual:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e8_ak/ae822fa0e5597xx0/rtss_he
   :goals: build flash

For the High-Performance core:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e8_ak/ae822fa0e5597xx0/rtss_hp
   :goals: build flash

Debugging
=========

Debug using J-Link debugger via JTAG/SWD interface.

References
**********

- `Alif Semiconductor Website <https://alifsemi.com/>`_
