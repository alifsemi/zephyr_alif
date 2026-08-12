# Copyright (C) 2024 Alif Semiconductor.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/crc.h)

zephyr_library_sources_ifdef(CONFIG_CRC_ALIF             alif_crc.c)
zephyr_library_sources_ifdef(CONFIG_USERSPACE            crc_handlers.c)
