# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if(CONFIG_RTSS_HE OR CONFIG_RTSS_HP)
  if(CONFIG_PM OR CONFIG_POWEROFF)
    zephyr_library_sources(rtss/power.c)
  endif()
endif()
