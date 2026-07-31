.. zephyr:code-sample:: usb-hid-joystick
   :name: USB HID joystick
   :relevant-api: _usb_device_core_api usb_hid_device_api input_interface

   Implement a basic HID joystick device.

Overview
********

This sample application demonstrates the use of the USB Human Interface Device
(HID) driver in Zephyr by implementing a USB HID Joystick.

The sample enumerates the board as a USB joystick and reports joystick button
and axis events to the host PC. The joystick is implemented using the Zephyr
:ref:`input` subsystem and a :dtcompatible:`gpio-keys` device defined in the
board devicetree.

This sample can be found under
:zephyr_file:`samples/subsys/usb/hid-joystick`
in the Zephyr project tree.

Requirements
************

This sample requires a USB device controller and uses the :ref:`input` API.

A :dtcompatible:`gpio-keys` device representing the joystick must be defined in
the board devicetree. The joystick should generate the following input events:

- ``INPUT_KEY_UP``: Move joystick up
- ``INPUT_KEY_DOWN``: Move joystick down
- ``INPUT_KEY_LEFT``: Move joystick left
- ``INPUT_KEY_RIGHT``: Move joystick right
- ``INPUT_KEY_ENTER``: Joystick center button

An LED must also be configured through the ``led0`` devicetree alias.
If the selected board does not define the required devicetree nodes, the sample
may build but will not function correctly.

Building and Running
********************

This sample can be built for supported boards. For example:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/hid-joystick
   :board: alif_e8_dk/ae822fa0e5597xx0/rtss_hp
   :snippets: alif-hid-joystick
   :goals: build
   :compact:

To build for the new experimental USB device stack, use:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/hid-joystick
   :board: alif_e8_dk/ae822fa0e5597xx0/rtss_hp
   :snippets: alif-hid-joystick
   :conf: usbd_next_prj.conf
   :goals: build
   :compact:

After building and flashing the sample, connect the board to a Linux host.

The device should enumerate as a USB HID joystick:

.. code-block:: console

    usb 3-3.1: New USB device found, idVendor=2fe3, idProduct=0008
    usb 3-3.1: Product: Alif HID Joystick
    usb 3-3.1: Manufacturer: Alif Semiconductor
    input: Alif Semiconductor Alif HID Joystick as /devices/.../input/inputX
    hid-generic ... USB HID v1.11 Joystick

You can verify joystick events using the Linux ``evtest`` utility:

.. code-block:: console

    sudo evtest /dev/input/eventX

or inspect the raw HID reports using:

.. code-block:: console

    sudo hexdump -Cv /dev/hidrawX

When the joystick is moved or the center button is pressed, corresponding HID
button and axis reports are sent to the host.
