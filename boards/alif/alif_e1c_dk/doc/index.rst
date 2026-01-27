.. _alif_e1c_dk:

Alif E1C Development Kit
########################

Overview
********

The Alif E1C Development Kit features the Ensemble E1C SoC, the entry-level device
in the Ensemble series. It provides an Arm Cortex-M55 processor with Helium vector
extensions and AI/ML acceleration capabilities in a compact, cost-optimized package.

.. note::
   The E1C is a cost-optimized, compact version of the Ensemble series device.

Hardware
********

- Alif Ensemble E1C SoC
- Arm Cortex-M55 processor with Helium vector extensions
- Arm Ethos-U55 NPU for AI/ML acceleration
- Integrated MRAM (on-chip non-volatile memory)
- USB connectivity
- Multiple GPIO, I2C, SPI, UART interfaces

.. note::
   The E1C SoC is available in multiple package variants:

   - Variants ending in ``hh``: CSP (Chip Scale Package)
   - Variants ending in ``ph``: BGA (Ball Grid Array) package

Serial Port
***********

The default console is UART2 configured at 115200 baud, 8N1.

Supported Features
==================

The alif_e1c_dk board configuration supports the following hardware features:

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

The Alif E1C DK supports flashing via JTAG/SWD using external debug probes.

Build and flash applications as usual:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: alif_e1c_dk/ae1c1f4051920hh/rtss_he
   :goals: build flash

Debugging
=========

Debug using J-Link debugger via JTAG/SWD interface.

References
**********

- `Alif Semiconductor Website <https://alifsemi.com/>`_
