# Copyright 2022 Arm Limited and/or its affiliates <open-source-office@arm.com>
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ARM_ETHOS_U AND CONFIG_MULTITHREADING)
    set(ETHOSU_TARGET_NPU_CONFIG ${CONFIG_ARM_ETHOS_U_NPU_NAME} CACHE STRING "")

    # Mapping log level from Zephyr (none=0, err=1, wrn=2, inf=3, dbg=4) to
    # Ethos-U driver (err=0, warn=1, info=2, debug=3)
    set(ETHOSU_LOG_SEVERITY_MAP err err warning info debug)
    list(LENGTH ETHOSU_LOG_SEVERITY_MAP ETHOSU_LOG_SEVERITY_MAP_LENGTH)

    if ("${CONFIG_ARM_ETHOS_U_LOG_LEVEL}" LESS_EQUAL "${ETHOSU_LOG_SEVERITY_MAP_LENGTH}")
        list(GET ETHOSU_LOG_SEVERITY_MAP ${CONFIG_ARM_ETHOS_U_LOG_LEVEL} ETHOSU_LOG_SEVERITY)
        set(ETHOSU_LOG_SEVERITY ${ETHOSU_LOG_SEVERITY} CACHE STRING "")
    else()
        set(ETHOSU_LOG_SEVERITY debug CACHE STRING "")
    endif()

    add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR} ethos-u-core-driver)

    if (CONFIG_SOC_FAMILY_ENSEMBLE AND NOT CONFIG_SOC_SERIES_ENSEMBLE_E1C)
        target_compile_definitions(ethosu_core_driver PRIVATE
            NPU_QCONFIG=1
            NPU_REGIONCFG_0=1
            NPU_REGIONCFG_1=0
            NPU_REGIONCFG_2=1
        )
    endif()

    target_link_libraries(ethosu_core_driver PUBLIC
       zephyr_interface)

    zephyr_link_libraries(ethosu_core_driver)
endif()
