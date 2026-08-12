# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_library_sources(soc_e1c.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/alif/ensemble/common/linker.ld CACHE INTERNAL "")
