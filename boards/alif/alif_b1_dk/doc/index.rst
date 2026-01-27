.. _alif_b1_dk:

Alif B1 Development Kit
#######################

Overview
********

The Alif B1 Development Kit is a development board featuring the Alif Semiconductor
Balletto B1 SoC with Arm Cortex-M55 processor.

Hardware
********

- Alif Balletto B1 SoC
- Arm Cortex-M55 processor with Helium vector extensions
- Arm Ethos-U55 NPU for AI/ML acceleration
- Integrated MRAM (on-chip non-volatile memory)
- USB connectivity
- Multiple GPIO, I2C, SPI, UART interfaces

.. note::
   The B1 SoC is available in multiple package variants:

   - Variants ending in ``hh``: CSP (Chip Scale Package)
   - Variants ending in ``ph``: BGA (Ball Grid Array) package

Serial Port
***********

The default console is UART2 configured at 115200 baud, 8N1.

Supported Features
==================

The alif_b1_dk board configuration supports the following hardware features:

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

The Alif B1 DK supports flashing via JTAG/SWD using external debug probes.

Build and flash applications as usual:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_b1_dk/ab1c1f4m51820ph0/rtss_he
   :goals: build flash

Debugging
=========

Debug using J-Link debugger via JTAG/SWD interface.

References
**********

- `Alif Semiconductor Website <https://alifsemi.com/>`_
