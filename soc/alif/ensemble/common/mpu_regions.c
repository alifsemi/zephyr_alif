/*
 * Copyright (c) 2025 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/mpu/arm_mpu.h>
#include <zephyr/devicetree.h>

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0 */
	MPU_REGION_ENTRY("FLASH_0", CONFIG_FLASH_BASE_ADDRESS,
			 REGION_FLASH_ATTR(CONFIG_FLASH_BASE_ADDRESS, CONFIG_FLASH_SIZE * 1024)),
	/* Region 1 */
	MPU_REGION_ENTRY("SRAM_0", CONFIG_SRAM_BASE_ADDRESS,
			 REGION_RAM_ATTR(CONFIG_SRAM_BASE_ADDRESS, CONFIG_SRAM_SIZE * 1024)),
	/* Region 2 */
	MPU_REGION_ENTRY("SRAM0", DT_REG_ADDR(DT_NODELABEL(sram0)),
			 REGION_RAM_ATTR(DT_REG_ADDR(DT_NODELABEL(sram0)),
			 DT_REG_SIZE(DT_NODELABEL(sram0)))),
	/* Region 3 */
	MPU_REGION_ENTRY("SRAM1", DT_REG_ADDR(DT_NODELABEL(sram1)),
			 REGION_RAM_ATTR(DT_REG_ADDR(DT_NODELABEL(sram1)),
			 DT_REG_SIZE(DT_NODELABEL(sram1)))),
	/* Region 4 */
	MPU_REGION_ENTRY("HOST_PERIPHERALS", DT_REG_ADDR(DT_NODELABEL(host_peripheral)),
			 REGION_DEVICE_ATTR(DT_REG_ADDR(DT_NODELABEL(host_peripheral)),
			 DT_REG_SIZE(DT_NODELABEL(host_peripheral)))),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
