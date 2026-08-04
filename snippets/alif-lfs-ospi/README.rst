.. _snippet-alif-lfs-ospi:

Alif LFS support over OSPI-Flash
#################################

Overview
********

This snippet enables LFS test app support over OSPI-Flash.

Devicetree layout
*****************

Alif RTSS boards normally define ``storage_partition`` under the
``mram_storage`` node.  That means the storage partition address is calculated
as an offset from the MRAM CPU-addressable base address, for example:

.. code-block:: devicetree

   mram_storage: mram_storage@80000000 {
           partitions {
                   storage_partition: partition@1A0000 {
                           reg = <0x001A0000 DT_SIZE_K(10)>;
                   };
           };
   };

When this snippet is enabled, the MRAM ``storage_partition`` node is deleted and
the same node label is re-created under ``flash_storage`` on OSPI flash:

.. code-block:: devicetree

   /delete-node/ &storage_partition;

   &flash_storage {
           partitions {
                   storage_partition: partition@0 {
                           label = "storage";
                           reg = <0x0 DT_SIZE_M(32)>;
                   };
           };
   };

The ``storage_partition`` node label is intentionally kept unchanged because the
LittleFS sample falls back to ``FIXED_PARTITION_ID(storage_partition)`` when no
``zephyr,fstab`` entry is provided.  Keeping the same label lets the application
use OSPI flash without source changes.

The snippet also updates the selected flash device:

.. code-block:: devicetree

   chosen {
           zephyr,flash-controller = &ospi_flash;
           zephyr,flash = &flash_storage;
   };

This makes the application-facing ``storage_partition`` refer to OSPI flash
storage while the MRAM node remains available for executable image partitions
such as ``boot_partition``, ``slot0_partition``, ``slot1_partition``, and
``scratch_partition``.

.. note::

   The OSPI flash capacity can vary between Alif devkits and fitted flash
   devices.  Update the ``storage_partition`` size in the OSPI overlay so it
   fits within the actual ``flash_storage`` device used by the target board.

Working with MRAM and OSPI storage
**********************************

The MRAM and OSPI flash areas are separate flash devices in devicetree:

* ``mram_storage`` is CPU-addressable MRAM and contains the executable image
  layout.
* ``flash_storage`` is the OSPI flash storage area used by this snippet.

Applications that need both areas should leave the MRAM partition names and
layout unchanged.  Any alternate partition label should be added only under
``flash_storage``.

For example, if the application needs a separate storage partition on OSPI flash

.. code-block:: devicetree

   &flash_storage {
           partitions {
                   my_storage_partition: partition@0 {
                           label = "my-storage";
                           reg = <0x0 DT_SIZE_M(32)>;
                   };
           };
   };

In this case the application can refer to ``FIXED_PARTITION_ID(my_storage_partition)``
or the ``zephyr,fstab`` label ``my-storage`` instead of ``storage_partition``.

Building and Running
********************

Example commands to build:

.. code-block:: console

   # E8 devkit
   west build -b alif_e8_dk/ae822fa0e5597xx0/rtss_he samples/subsys/fs/littlefs -S alif-lfs-ospi -S ospi-flash -p

   # E7 devkit
   west build -b alif_e7_dk/ae722f80f55d5xx/rtss_he samples/subsys/fs/littlefs -S alif-lfs-ospi -S ospi-flash -p

   # B1 devkit
   west build -b alif_b1_dk/ab1c1f4m51820hh0/rtss_he samples/subsys/fs/littlefs -S alif-lfs-ospi -S ospi-flash -p

The :ref:`snippet-ospi-flash` snippet provides the board connected
OSPI-Flash pinctrl information, so it should be included along with the
``alif-lfs-ospi`` application snippet when building this sample.

The application can be found under :zephyr_file:`samples/subsys/fs/littlefs` in the Zephyr tree.

See :zephyr:code-sample:`fs/littlefs` application details.
