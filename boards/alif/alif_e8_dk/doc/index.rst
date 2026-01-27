.. _alif_e8_dk:

Alif E8 Development Kit
#######################

Overview
********

The Alif E8 Development Kit is the primary development platform for the Ensemble
E4, E6, and E8 SoC family, featuring dual Arm Cortex-M55 processors, one Ethos-U85 NPU
for Generative AI, and dual Ethos-U55 NPUs.

.. note::
   This development kit supports the full E4/E6/E8 SoC family. The E4 and E6 are
   cost-optimized variants with reduced features compared to the E8.

   The E8 supports both RTSS-HE (High-Efficiency) and RTSS-HP (High-Performance)
   Cortex-M55 core targets.

Hardware
********

- Alif Ensemble E8 SoC
- Dual Arm Cortex-M55 processors (High-Performance + High-Efficiency)
- 1× Arm Ethos-U85 NPU for Generative AI (256 MAC/cycle, up to 204 GOPS)
- 2× Arm Ethos-U55 NPUs (256 MAC/cycle + 128 MAC/cycle)
- Integrated MRAM (on-chip non-volatile memory)
- USB connectivity
- Multiple GPIO, I2C, SPI, UART interfaces

Serial Port
***********

The default console UART varies by core:

- RTSS-HE (High-Efficiency): UART2 at 115200 baud, 8N1
- RTSS-HP (High-Performance): UART4 at 115200 baud, 8N1

Supported Features
==================

The alif_e8_dk board configuration supports the following hardware features:

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

The Alif E8 DK supports flashing via JTAG/SWD using external debug probes.

Build and flash applications as usual:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e8_dk/ae822fa0e5597xx0/rtss_he
   :goals: build flash

For the High-Performance core:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e8_dk/ae822fa0e5597xx0/rtss_hp
   :goals: build flash

Debugging
=========

Debug using J-Link debugger via JTAG/SWD interface.

References
**********

- `Alif Semiconductor Website <https://alifsemi.com/>`_
