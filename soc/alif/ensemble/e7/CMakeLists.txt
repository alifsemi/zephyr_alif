# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

#zephyr_library_sources(soc.c)
zephyr_include_directories(.)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/alif/ensemble/common/linker.ld CACHE INTERNAL "")
