# Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(./)

zephyr_library_sources_ifdef(CONFIG_WIFI_AIROC
  airoc_wifi.c
  airoc_whd_hal_common.c
)
zephyr_library_sources_ifdef(CONFIG_AIROC_WIFI_BUS_SDIO airoc_whd_hal_sdio.c)
zephyr_library_sources_ifdef(CONFIG_AIROC_WIFI_BUS_SPI airoc_whd_hal_spi.c)
zephyr_library_sources_ifdef(CONFIG_AIROC_WIFI_P2P
  airoc_wifi_p2p.c
  airoc_wifi_p2p_ie.c
  airoc_wifi_wps_reg.c
)
if(CONFIG_AIROC_WIFI_P2P AND CONFIG_MBEDTLS)
  zephyr_library_include_directories(
    ${ZEPHYR_MBEDTLS_MODULE_DIR}/include
    ${ZEPHYR_BASE}/modules/mbedtls/configs
    ${ZEPHYR_BASE}/modules/mbedtls/include
  )
  zephyr_library_compile_definitions(
    MBEDTLS_CONFIG_FILE="config-tls-generic.h"
  )
endif()

zephyr_compile_definitions(CYBSP_WIFI_CAPABLE)
zephyr_compile_definitions(CY_RTOS_AWARE)
zephyr_compile_definitions(WHD_USE_CUSTOM_MALLOC_IMPL)
zephyr_compile_definitions(WHD_USE_CUSTOM_HAL_IMPL)
