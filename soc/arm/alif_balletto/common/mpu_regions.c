/*
 * Copyright (c) 2024 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/slist.h>
#include <zephyr/arch/arm/mpu/arm_mpu.h>

#define ALIF_HOST_OSPI_REG		0x83000000
#define ALIF_HOST_OSPI_SIZE		KB(16)

#define ALIF_HOST_PERIPHERAL_BASE	0x1A000000
#define ALIF_HOST_PERIPHERAL_SIZE	MB(16)

#define ALIF_HOST_OSPI0_XIP_BASE	0xA0000000
#define ALIF_HOST_OSPI0_XIP_SIZE	MB(512)

#define REGION_OSPI_FLASH_ATTR(base, size) \
{\
	.rbar = RO_Msk | NON_SHAREABLE_Msk, \
	/* Cache-ability */ \
	.mair_idx = MPU_MAIR_INDEX_SRAM_NOCACHE, \
	.r_limit = REGION_LIMIT_ADDR(base, size),  \
}

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0 */
	MPU_REGION_ENTRY("FLASH_0", CONFIG_FLASH_BASE_ADDRESS,
			 REGION_FLASH_ATTR(CONFIG_FLASH_BASE_ADDRESS, CONFIG_FLASH_SIZE * 1024)),
	/* Region 1 */
	MPU_REGION_ENTRY("SRAM_0", CONFIG_SRAM_BASE_ADDRESS,
			 REGION_RAM_ATTR(CONFIG_SRAM_BASE_ADDRESS, CONFIG_SRAM_SIZE * 1024)),
	/* Region 2 */
	MPU_REGION_ENTRY("OSPI_CTRL", ALIF_HOST_OSPI_REG,
			 REGION_DEVICE_ATTR(ALIF_HOST_OSPI_REG, ALIF_HOST_OSPI_SIZE)),
	/* Region 3 */
	MPU_REGION_ENTRY("PERIPHERALS", ALIF_HOST_PERIPHERAL_BASE,
			 REGION_DEVICE_ATTR(ALIF_HOST_PERIPHERAL_BASE,
							ALIF_HOST_PERIPHERAL_SIZE)),
	/* Region 4 */
	MPU_REGION_ENTRY("OSPI0_XIP", ALIF_HOST_OSPI0_XIP_BASE,
			 REGION_OSPI_FLASH_ATTR(ALIF_HOST_OSPI0_XIP_BASE,
							ALIF_HOST_OSPI0_XIP_SIZE)),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
