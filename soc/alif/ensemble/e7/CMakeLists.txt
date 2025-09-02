# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if(CONFIG_APSS)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/alif/ensemble/common/linker_a.ld CACHE INTERNAL "")
else()
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/alif/ensemble/common/linker.ld CACHE INTERNAL "")
endif()