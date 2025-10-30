.. _snippet-alif-lfs-mram:

Alif LittleFS support over MRAM
#################################

Overview
********

This snippet enables LFS test app support over MRAM

Building and Running
********************

Example command to build:

.. code-block:: console

   west build -b alif_b1_dk/ab1c1f4m51820hh/rtss_he -S alif-lfs-mram samples/subsys/fs/littlefs/ -p
   OR
   west build -b alif_b1_dk/ab1c1f4m51820hh/rtss_he samples/subsys/fs/littlefs/ -p -- -DSNIPPET=alif-lfs-mram

The application can be found under :zephyr_file:`samples/subsys/fs/littlefs` in the Zephyr tree.

See :zephyr:code-sample:`fs/littlefs` application details.
