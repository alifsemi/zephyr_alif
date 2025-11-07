.. _snippet-alif-lfs-ospi:

Alif LFS support over OSPI-Flash
#################################

Overview
********

This snippet enables LFS test app support over OSPI-Flash

Building and Running
********************

Example command to build:

.. code-block:: console

   west build -b alif_b1_dk//rtss_he samples/subsys/fs/littlefs -p -- -DSNIPPET=alif-lfs-ospi

The application can be found under :zephyr_file:`samples/subsys/fs/littlefs` in the Zephyr tree.

See :zephyr:code-sample:`fs/littlefs` application details.


