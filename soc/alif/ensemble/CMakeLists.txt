# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/common)

zephyr_include_directories(common)
zephyr_sources(common/soc_common.c)

zephyr_sources_ifdef(CONFIG_ARM_AARCH32_MMU common/mmu_regions.c)
zephyr_sources_ifdef(CONFIG_APSS common/vector_table.c)

if(CONFIG_RTSS_HE OR CONFIG_RTSS_HP)
  zephyr_sources(common/startup.S)
endif()

if(CONFIG_ARM_MPU AND CONFIG_CPU_HAS_CUSTOM_FIXED_SOC_MPU_REGIONS)
  zephyr_sources(common/mpu_regions.c)
endif()

if(CMAKE_C_COMPILER_ID STREQUAL "ARMClang")
  zephyr_linker_section_configure(SECTION ${CONFIG_NONSECURE0_REGION} INPUT ".alif_ns" )
  zephyr_linker_section_configure(SECTION ${CONFIG_NONSECURE0_REGION} INPUT ".alif_ns.*" )
if(CONFIG_SRAM0_REGION)
  zephyr_linker_section_configure(SECTION ${CONFIG_SRAM0_REGION} INPUT ".alif_sram0" )
  zephyr_linker_section_configure(SECTION ${CONFIG_SRAM0_REGION} INPUT ".alif_sram0.*" )
endif()
if(CONFIG_SRAM1_REGION)
  zephyr_linker_section_configure(SECTION ${CONFIG_SRAM1_REGION} INPUT ".alif_sram1" )
  zephyr_linker_section_configure(SECTION ${CONFIG_SRAM1_REGION} INPUT ".alif_sram1.*" )
endif()
if(CONFIG_OSPI0_XIP_REGION)
  zephyr_linker_section_configure(SECTION ${CONFIG_OSPI0_XIP_REGION} INPUT ".alif_extflash_OSPI0" )
  zephyr_linker_section_configure(SECTION ${CONFIG_OSPI0_XIP_REGION} INPUT ".alif_extflash_OSPI0.*" )
endif()
if(CONFIG_OSPI1_XIP_REGION)
  zephyr_linker_section_configure(SECTION ${CONFIG_OSPI1_XIP_REGION} INPUT ".alif_extflash_OSPI1" )
  zephyr_linker_section_configure(SECTION ${CONFIG_OSPI1_XIP_REGION} INPUT ".alif_extflash_OSPI1.*" )
endif()
else()
  zephyr_linker_sources(SECTIONS common/alif_ns.ld)
  zephyr_linker_sources(SECTIONS common/alif_ospi.ld)
endif()
