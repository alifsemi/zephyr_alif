# Copyright (c) 2024 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
zephyr_include_directories(common)

zephyr_sources(common/soc.c)
zephyr_sources_ifdef(CONFIG_PM_RTSS
  common/power_rtss.c
  )

if(CONFIG_ARM_MPU AND CONFIG_CPU_HAS_CUSTOM_FIXED_SOC_MPU_REGIONS)

  zephyr_library()

  zephyr_library_sources_ifdef(CONFIG_CPU_HAS_ARM_MPU
    common/mpu_regions.c
  )
endif()
