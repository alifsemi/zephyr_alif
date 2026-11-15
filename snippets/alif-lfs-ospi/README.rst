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

When this snippet is enabled, the MRAM ``storage_partition`` remains unchanged.
A separate LittleFS partition is created under ``flash_storage`` on OSPI flash:

.. code-block:: devicetree

   &flash_storage {
           partitions {
                   lfs1_partition: partition@0 {
                           label = "lfs1";
                           reg = <0x0 DT_SIZE_M(32)>;
                   };
           };
   };

The snippet adds a ``zephyr,fstab`` entry that directs the LittleFS sample to the
OSPI partition:

.. code-block:: devicetree

   fstab {
           compatible = "zephyr,fstab";

           lfs1: lfs1 {
                   compatible = "zephyr,fstab,littlefs";
                   mount-point = "/lfs";
                   partition = <&lfs1_partition>;
                   read-size = <16>;
                   prog-size = <16>;
                   cache-size = <64>;
                   lookahead-size = <32>;
                   block-cycles = <512>;
           };
   };

The snippet also updates the selected flash device:

.. code-block:: devicetree

   chosen {
           zephyr,flash-controller = &ospi_flash;
           zephyr,flash = &flash_storage;
   };

This lets the application use OSPI storage without changing the MRAM partition
layout or the MPU regions derived from it.

.. note::

   The OSPI flash capacity can vary between Alif devkits and fitted flash
   devices.  Update the ``lfs1_partition`` size in the OSPI overlay so it
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

For example, an application can add another partition on OSPI flash

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
or add another ``zephyr,fstab`` entry for that partition.

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
