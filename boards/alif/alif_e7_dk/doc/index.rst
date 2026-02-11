.. _alif_e7_dk:

Alif E7 Development Kit
#######################

Overview
********

The Alif E7 Development Kit is the primary development platform for the Ensemble
E3, E5, and E7 SoC family, featuring dual Arm Cortex-M55 processors and dual
Ethos-U55 NPUs.

.. note::
   This development kit supports the full E3/E5/E7 SoC family. The E3 and E5 are
   cost-optimized variants with reduced features compared to the E7.

   The E7 supports both RTSS-HE (High-Efficiency) and RTSS-HP (High-Performance)
   Cortex-M55 core targets.

Board Identifiers
*****************

The following board qualifiers are supported:

+----------------------------------------------+----------+------------------+
| Board Identifier                             | SoC      | Core             |
+==============================================+==========+==================+
| ``alif_e7_dk/ae722f80f55d5xx/rtss_he``       | E7       | RTSS-HE          |
+----------------------------------------------+----------+------------------+
| ``alif_e7_dk/ae722f80f55d5xx/rtss_hp``       | E7       | RTSS-HP          |
+----------------------------------------------+----------+------------------+
| ``alif_e7_dk/ae722f80f55d5xx/apss``          | E7       | APSS             |
+----------------------------------------------+----------+------------------+
| ``alif_e7_dk/ae302f80f55d5xx/rtss_he``       | E3       | RTSS-HE          |
+----------------------------------------------+----------+------------------+
| ``alif_e7_dk/ae302f80f55d5xx/rtss_hp``       | E3       | RTSS-HP          |
+----------------------------------------------+----------+------------------+

Hardware
********

- Alif Ensemble E7 SoC
- Dual Arm Cortex-M55 processors (High-Performance + High-Efficiency)
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

The alif_e7_dk board configuration supports the following hardware features:

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

The Alif E7 DK supports flashing via JTAG/SWD using external debug probes.

Build and flash applications using any of the board identifiers listed above.
For example:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e7_dk/ae722f80f55d5xx/rtss_he
   :goals: build flash

For the High-Performance core:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e7_dk/ae722f80f55d5xx/rtss_hp
   :goals: build flash

Debugging
=========

Debug using J-Link debugger via JTAG/SWD interface.

References
**********

- `Alif Semiconductor Website <https://alifsemi.com/>`_
