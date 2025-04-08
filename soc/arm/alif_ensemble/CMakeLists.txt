# Copyright (c) 2024 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
zephyr_include_directories(common)

zephyr_sources(common/soc.c)
zephyr_sources_ifdef(CONFIG_PM_RTSS
  common/power_rtss.c
  )


if(CMAKE_C_COMPILER_ID STREQUAL "ARMClang")
if(CONFIG_ARM_SECURE_FIRMWARE)
  zephyr_linker_section_configure(SECTION ${CONFIG_NONSECURE0_REGION} INPUT ".alif_ns" )
  zephyr_linker_section_configure(SECTION ${CONFIG_NONSECURE0_REGION} INPUT ".alif_ns.*" )
endif()
if(CONFIG_SRAM0_REGION)
  zephyr_linker_section_configure(SECTION ${CONFIG_SRAM0_REGION} INPUT ".alif_sram0" )
  zephyr_linker_section_configure(SECTION ${CONFIG_SRAM0_REGION} INPUT ".alif_sram0.*" )
endif()
if(CONFIG_SRAM1_REGION)
  zephyr_linker_section_configure(SECTION ${CONFIG_SRAM1_REGION} INPUT ".alif_sram1" )
  zephyr_linker_section_configure(SECTION ${CONFIG_SRAM1_REGION} INPUT ".alif_sram1.*" )
endif()
else()
  zephyr_linker_sources_ifdef(CONFIG_ARM_SECURE_FIRMWARE SECTIONS common/alif_ns.ld)
  zephyr_linker_sources(SECTIONS common/alif_sram.ld)
endif()


if(CONFIG_PM_RTSS AND CONFIG_POWEROFF)
  zephyr_sources(common/poweroff_rtss.c)
endif()

if(CONFIG_ARM_MPU AND CONFIG_CPU_HAS_CUSTOM_FIXED_SOC_MPU_REGIONS)

  zephyr_library()

  zephyr_library_sources_ifdef(CONFIG_CPU_HAS_ARM_MPU
    common/mpu_regions.c
  )
endif()
