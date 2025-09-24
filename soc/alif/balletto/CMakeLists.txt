# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/common)
zephyr_include_directories(common)
zephyr_sources(common/startup.S)
zephyr_sources(common/soc_common.c)

if(CONFIG_ARM_MPU AND CONFIG_CPU_HAS_CUSTOM_FIXED_SOC_MPU_REGIONS)
  zephyr_sources(common/mpu_regions.c)
endif()

if(CMAKE_C_COMPILER_ID STREQUAL "ARMClang")
  zephyr_linker_section_configure(SECTION ${CONFIG_NONSECURE0_REGION} INPUT ".alif_ns" )
  zephyr_linker_section_configure(SECTION ${CONFIG_NONSECURE0_REGION} INPUT ".alif_ns.*" )
else()
  zephyr_linker_sources(SECTIONS common/alif_ns.ld)
endif()
