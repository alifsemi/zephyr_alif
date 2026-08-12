/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_POWER_DOMAIN_ALIF_POWER_DOMAIN_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_POWER_DOMAIN_ALIF_POWER_DOMAIN_H_

/**
 * @brief Alif Power Domain IDs
 *
 * These IDs correspond to the bit positions in the SE Services
 * power_domains field. The driver will convert these to masks
 * using BIT(id).
 */

#define ALIF_PD_VBAT_AON       0  /* VBAT Always-On */
#define ALIF_PD_SRAM_CTRL_AON  1  /* SRAM Controller AON */
#define ALIF_PD_SSE700_AON     2  /* SSE-700 AON */
#define ALIF_PD_RTSS_HE        3  /* RTSS High Efficiency */
#define ALIF_PD_SRAMS          4  /* SRAMs */
#define ALIF_PD_SESS           5  /* Secure Enclave */
#define ALIF_PD_SYST           6  /* System Top (UART, SPI, I2C, etc.) */
#define ALIF_PD_RTSS_HP        7  /* RTSS High Performance */
#define ALIF_PD_DBSS           8  /* Debug Subsystem */
#define ALIF_PD_APPS           9  /* Applications */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_POWER_DOMAIN_ALIF_POWER_DOMAIN_H_ */
