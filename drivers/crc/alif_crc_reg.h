/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CRC_ALIF_CRC_REG_H_
#define ZEPHYR_DRIVERS_CRC_ALIF_CRC_REG_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Device data structure. Includes:
 * 1. CRC algorithm : 8-bit, 16-bit, 32-bit
 */
struct crc_data {
	uint8_t crc_algo;
};

/*
 * Device config structure. Includes:
 *	1. Device MMIO information
 */
struct crc_config {
	DEVICE_MMIO_ROM;
};

#define CRC_CONTROL         0x00	/* CRC Calculation Setup Register */
#define CRC_SEED            0x10	/* Seed Value Register            */
#define CRC_POLY_CUSTOM     0x14	/* Custom Polynomial Register     */
#define CRC_OUT             0x18	/* Accumulated CRC Register       */
#define CRC_DATA_IN_8_0     0x20	/* 8-bit Values Register          */
#define CRC_DATA_IN_32_0    0x60	/* 32-bit Values Register         */

#define CRC_8_BIT_SIZE            0
#define CRC_16_BIT_SIZE           1
#define CRC_32_BIT_SIZE           2

#define CRC_8_CCITT              (0 << 3)	/* To select the CRC_8_CCITT           */
#define CRC_ALGO_8_BIT_SIZE      (0 << 1)	/* To select the 8 bit algorithm size  */
#define CRC_ALGO_16_BIT_SIZE     (1 << 1)	/* To select the 16 bit algorithm size */
#define CRC_16_CCITT             (3 << 3)	/* To select the CRC_16_CCITT          */
#define CRC_INIT_BIT              BIT(0)	/* To select the init value            */
#define CRC_ALGO_32_BIT_SIZE     (2 << 1)	/* To select the 32 bit algorithm size */
#define CRC_BIT_SWAP             (1 << 8)	/* To enable the Bit swap              */
#define CRC_BYTE_SWAP            (1 << 7)	/* To enable the Byte swap             */
#define CRC_REFLECT              (1 << 11)	/* To Reflect the CRC value            */
#define CRC_INVERT               (1 << 10)	/* To Invert the CRC value             */
#define CRC_32                   (4 << 3)	/* To select the CRC_32                */
#define CRC_32C                  (5 << 3)	/* To select the CRC_32C               */
#define CRC_CUSTOM_POLY          (1 << 9)	/* To enable the poly custom           */

#define CRC_STANDARD_POLY       0x04C11DB7	/* Standard polynomial for 32 bit CRC  */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CRC_ALIF_CRC_REG_H_ */
