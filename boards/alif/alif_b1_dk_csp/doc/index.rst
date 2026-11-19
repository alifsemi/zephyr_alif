.. _alif_b1_dk_csp:

Alif B1 DK CSP HPA
##################

Overview
********

The Alif B1 DK CSP HPA (``alif_b1_dk_csp``) is the WLCSP120 Balletto B1
development kit, part ``AB1C1F4M51820HH0``.

The WLCSP package does not bond port 8. This board has no Click socket,
no MIPI panel, and no LP-camera sync pins. ``gpio8`` is not present.
The default pin groups for UART0, UART3, SPI1, CAN0, and the LP camera
were on port 8, so this board does not provide those groups. The fitted
CAN transceiver is CAN1 on P1_3, P1_4, and P1_5. UART2 is the console.

The green user LED (``led0``) is P4_5 (``gpio4`` pin 5).

Hardware
********

- Alif Balletto B1 SoC, WLCSP120, ``AB1C1F4M51820HH0``
- Arm Cortex-M55 RTSS-HE with Helium
- Arm Ethos-U55 NPU
- On-chip MRAM
- USB, GPIO, I2C, SPI, and UART

Serial Port
***********

The default console is UART2 at 115200 baud, 8N1.

Supported Features
==================

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

Blink the green LED:

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: alif_b1_dk_csp/ab1c1f4m51820hh0/rtss_he
   :goals: build flash

Debug using J-Link via JTAG/SWD.

References
**********

- `Alif Semiconductor Website <https://alifsemi.com/>`_
