# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
zephyr_include_directories(common)
zephyr_include_directories(../common)
zephyr_sources(common/soc_common.c)

zephyr_sources_ifdef(CONFIG_ARM_AARCH32_MMU common/mmu_regions.c)
zephyr_sources_ifdef(CONFIG_APSS common/vector_table.c)

if(CONFIG_RTSS_HE OR CONFIG_RTSS_HP)
  zephyr_sources(common/startup.S)
endif()

if(CONFIG_ARM_MPU AND CONFIG_CPU_HAS_CUSTOM_FIXED_SOC_MPU_REGIONS)
  zephyr_sources(common/mpu_regions.c)
endif()
