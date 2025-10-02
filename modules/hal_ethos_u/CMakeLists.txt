# SPDX-FileCopyrightText: <text>Copyright 2022, 2024 Arm Limited and/or its
# affiliates <open-source-office@arm.com></text>
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ARM_ETHOS_U AND CONFIG_MULTITHREADING)
    set(ETHOSU_TARGET_NPU_CONFIG ${CONFIG_ARM_ETHOS_U_NPU_NAME})

    # Mapping log level from Zephyr (none=0, err=1, wrn=2, inf=3, dbg=4) to
    # Ethos-U driver (err=0, warn=1, info=2, debug=3)
    set(ETHOSU_LOG_SEVERITY_MAP err err warning info debug)
    list(LENGTH ETHOSU_LOG_SEVERITY_MAP ETHOSU_LOG_SEVERITY_MAP_LENGTH)

    if (${CONFIG_ARM_ETHOS_U_LOG_LEVEL} EQUAL 0)
        # The Ethos-U driver does not have a corresponding "none" log level. Disable logging instead.
        set(ETHOSU_LOG_ENABLE OFF CACHE BOOL "")
    elseif (${CONFIG_ARM_ETHOS_U_LOG_LEVEL} LESS "${ETHOSU_LOG_SEVERITY_MAP_LENGTH}")
        list(GET ETHOSU_LOG_SEVERITY_MAP ${CONFIG_ARM_ETHOS_U_LOG_LEVEL} ETHOSU_LOG_SEVERITY)
        set(ETHOSU_LOG_SEVERITY ${ETHOSU_LOG_SEVERITY} CACHE STRING "")
    else()
        set(ETHOSU_LOG_SEVERITY debug CACHE STRING "")
    endif()

    add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR} ethos-u-core-driver)

    if (CONFIG_SOC_FAMILY_ENSEMBLE)
        if(NOT (CONFIG_SOC_SERIES_E1C OR CONFIG_ENSEMBLE_GEN2))
            target_compile_definitions(ethosu_core_driver PRIVATE
                NPU_QCONFIG=1
                NPU_REGIONCFG_0=1
                NPU_REGIONCFG_1=0
                NPU_REGIONCFG_2=1
            )
        endif()
    endif()

    if(ETHOSU_TARGET_NPU_CONFIG MATCHES "^ethos-(u[0-9]+)-([0-9]+)$")
        set(ETHOSU_ARCH "${CMAKE_MATCH_1}")
        set(ETHOSU_MACS "${CMAKE_MATCH_2}")
        # Ethos-U55 and Ethos-U65 supports up to 128 bytes,
        # and Ethos-U85 supports up to 256 bytes.
        # Although, this is system implementation dependent the platforms we build for should support the
        # maximum burst length for all NPU configurations.
        if (ETHOSU_ARCH STREQUAL "u55" OR ETHOSU_ARCH STREQUAL "u65")
            target_compile_definitions(ethosu_core_driver PUBLIC
                AXI_LIMIT0_MAX_BEATS_BYTES=1
                AXI_LIMIT1_MAX_BEATS_BYTES=1
                AXI_LIMIT2_MAX_BEATS_BYTES=1
                AXI_LIMIT3_MAX_BEATS_BYTES=1) # 0 = 64 byte burst & 1 = 128 byte burst
            # Use "normal" (modifiable) transactions, improving interconnect transit
            target_compile_definitions(ethosu_core_driver PUBLIC
                AXI_LIMIT0_MEM_TYPE=2
                AXI_LIMIT1_MEM_TYPE=2
                AXI_LIMIT2_MEM_TYPE=2
                AXI_LIMIT3_MEM_TYPE=2) # 2 = Normal Non-cacheable Non-bufferable
        elseif (ETHOSU_ARCH STREQUAL "u85")
            target_compile_definitions(ethosu_core_driver PRIVATE
                AXI_LIMIT_SRAM_MAX_BEATS=2
                AXI_LIMIT_EXT_MAX_BEATS=2) # 0 = 64 byte burst, 1 = 128 byte burst, 2 = 256 byte burst
            # Use "normal" (modifiable) transactions, improving interconnect transit
            target_compile_definitions(ethosu_core_driver PRIVATE
                NPU_MEM_ATTR_0=0x20  # Normal Non-cacheable Non-bufferable, SRAM ports
                NPU_MEM_ATTR_1=0x20  # Normal Non-cacheable Non-bufferable, SRAM ports
                NPU_MEM_ATTR_2=0x24  # Normal Non-cacheable Non-bufferable, EXT ports
                NPU_MEM_ATTR_3=0x24) # Normal Non-cacheable Non-bufferable, EXT ports
        endif()
    endif()

    target_link_libraries(ethosu_core_driver PUBLIC
       zephyr_interface)

    zephyr_link_libraries(ethosu_core_driver)
endif()
