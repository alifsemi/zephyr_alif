# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(udc_common.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/usb/common/)

zephyr_library_sources_ifdef(CONFIG_UDC_SKELETON udc_skeleton.c)
zephyr_library_sources_ifdef(CONFIG_UDC_VIRTUAL udc_virtual.c)
zephyr_library_sources_ifdef(CONFIG_UDC_DWC3 udc_dwc3.c)
