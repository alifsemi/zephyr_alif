# Copyright (c) 2024 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
zephyr_include_directories(common)

zephyr_sources(common/soc_balletto_common.S)
zephyr_sources_ifdef(CONFIG_PM_RTSS
  common/power_rtss.c
  )

zephyr_sources_ifdef(CONFIG_ARM_SECURE_FIRMWARE common/tgu_M55.c)

if(CMAKE_C_COMPILER_ID STREQUAL "ARMClang")
if(CONFIG_ARM_SECURE_FIRMWARE)
  zephyr_linker_section_configure(SECTION ${CONFIG_NONSECURE0_REGION} INPUT ".alif_ns" )
  zephyr_linker_section_configure(SECTION ${CONFIG_NONSECURE0_REGION} INPUT ".alif_ns.*" )
endif()
if(CONFIG_OSPI0_XIP_REGION)
  zephyr_linker_section_configure(SECTION ${CONFIG_OSPI0_XIP_REGION} INPUT ".alif_extflash_OSPI0" )
  zephyr_linker_section_configure(SECTION ${CONFIG_OSPI0_XIP_REGION} INPUT ".alif_extflash_OSPI0.*" )
endif()
else()
  zephyr_linker_sources_ifdef(CONFIG_ARM_SECURE_FIRMWARE SECTIONS common/alif_ns.ld)
  zephyr_linker_sources(SECTIONS common/alif_ospi.ld)
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

# If bin output and external flash XIP mode is enabled, split the output to separate .bin files
if(CONFIG_BUILD_OUTPUT_BIN AND CONFIG_ALIF_OSPI_FLASH_XIP)

  if(NOT CONFIG_CPP_EXCEPTIONS)
    set(eh_frame_section ".eh_frame")
  else()
    set(eh_frame_section "")
  endif()

  set(remove_sections_argument_list "")
  foreach(section .comment COMMON ${eh_frame_section})
    list(APPEND remove_sections_argument_list
      $<TARGET_PROPERTY:bintools,elfconvert_flag_section_remove>${section})
  endforeach()

  if(CONFIG_OSPI0_XIP_REGION)
    list(APPEND remove_sections_argument_list
      $<TARGET_PROPERTY:bintools,elfconvert_flag_section_remove>${CONFIG_OSPI0_XIP_REGION})

    # Extract the OSPI0 external flash section to its own binary
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND $<TARGET_PROPERTY:bintools,elfconvert_command>
      $<TARGET_PROPERTY:bintools,elfconvert_flag>
      $<TARGET_PROPERTY:bintools,elfconvert_flag_outtarget>binary
      $<TARGET_PROPERTY:bintools,elfconvert_flag_section_only>${CONFIG_OSPI0_XIP_REGION}
      $<TARGET_PROPERTY:bintools,elfconvert_flag_infile>${CMAKE_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
      $<TARGET_PROPERTY:bintools,elfconvert_flag_outfile>${CMAKE_BINARY_DIR}/zephyr/${CONFIG_OSPI0_XIP_REGION}.bin
      $<TARGET_PROPERTY:bintools,elfconvert_flag_final>
    )
  endif()

  # Extract the main binary without the external flash part
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND $<TARGET_PROPERTY:bintools,elfconvert_command>
    $<TARGET_PROPERTY:bintools,elfconvert_flag>
    $<TARGET_PROPERTY:bintools,elfconvert_flag_outtarget>binary
    ${remove_sections_argument_list}
    $<TARGET_PROPERTY:bintools,elfconvert_flag_infile>${CMAKE_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
    $<TARGET_PROPERTY:bintools,elfconvert_flag_outfile>${CMAKE_BINARY_DIR}/zephyr/${KERNEL_BIN_NAME}
    $<TARGET_PROPERTY:bintools,elfconvert_flag_final>
  )

  # Disable the default binary creation
  get_property(elfconvert_formats TARGET bintools PROPERTY elfconvert_formats)
  list(REMOVE_ITEM
      elfconvert_formats
      binary
  )
  set_property(TARGET bintools PROPERTY elfconvert_formats)

endif()
