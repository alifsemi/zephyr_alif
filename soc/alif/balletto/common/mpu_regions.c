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

#define MRAM_SECTOR_SIZE		DT_PROP(DT_NODELABEL(mram_storage), erase_block_size)

#define MRAM_BOOT_PARTITION_ADDR	DT_FIXED_PARTITION_ADDR(DT_NODELABEL(boot_partition))
#define MRAM_BOOT_PARTITION_SIZE	DT_REG_SIZE(DT_NODELABEL(boot_partition))
#define MRAM_SLOT0_PARTITION_ADDR	DT_FIXED_PARTITION_ADDR(DT_NODELABEL(slot0_partition))
#define MRAM_SLOT0_PARTITION_SIZE	DT_REG_SIZE(DT_NODELABEL(slot0_partition))
#define MRAM_SLOT1_PARTITION_ADDR	DT_FIXED_PARTITION_ADDR(DT_NODELABEL(slot1_partition))
#define MRAM_SLOT1_PARTITION_SIZE	DT_REG_SIZE(DT_NODELABEL(slot1_partition))
#define MRAM_SCRATCH_PARTITION_ADDR	DT_FIXED_PARTITION_ADDR(DT_NODELABEL(scratch_partition))
#define MRAM_SCRATCH_PARTITION_SIZE	DT_REG_SIZE(DT_NODELABEL(scratch_partition))
#define MRAM_STORAGE_PARTITION_ADDR	DT_FIXED_PARTITION_ADDR(DT_NODELABEL(storage_partition))
#define MRAM_STORAGE_PARTITION_SIZE	DT_REG_SIZE(DT_NODELABEL(storage_partition))

#ifdef CONFIG_MCUBOOT
/*
 * MCUBoot bootloader builds
 * MCUBoot executes from boot partition.
 */
#define FLASH_MRAM_BASE_ADDR	MRAM_BOOT_PARTITION_ADDR
#define FLASH_MRAM_SIZE		MRAM_BOOT_PARTITION_SIZE
/* Writable regions: slot0, slot1, scratch, storage */
#define DEVICE_MRAM_BASE_ADDR	MRAM_SLOT0_PARTITION_ADDR
#define DEVICE_MRAM_SIZE	(MRAM_STORAGE_PARTITION_ADDR + MRAM_STORAGE_PARTITION_SIZE \
						- MRAM_SLOT0_PARTITION_ADDR)
#elif defined(CONFIG_BOOTLOADER_MCUBOOT)
/*
 * MCUBoot-compatible app builds
 * Application executes from slot0, needs to write trailer for swap operations.
 * boot and slot0 (minus trailer sector) are executable.
 * The last sector of slot0 is reserved as writable device memory for MCUBoot API
 * trailer writes (boot_write_img_confirmed, boot_request_upgrade, etc.). Though the
 * actual app-writable trailer area is much less than a sector, the entire last
 * sector is reserved to align the MPU region on a sector boundary.
 */
#define FLASH_MRAM_BASE_ADDR	MRAM_BOOT_PARTITION_ADDR
#define FLASH_MRAM_SIZE		(MRAM_SLOT0_PARTITION_ADDR + MRAM_SLOT0_PARTITION_SIZE - \
				 MRAM_BOOT_PARTITION_ADDR - MRAM_SECTOR_SIZE)
/* Writable regions: slot0 trailer sector, slot1, scratch, storage */
#define DEVICE_MRAM_BASE_ADDR	(MRAM_SLOT0_PARTITION_ADDR + MRAM_SLOT0_PARTITION_SIZE \
						- MRAM_SECTOR_SIZE)
#define DEVICE_MRAM_SIZE	(MRAM_STORAGE_PARTITION_ADDR + MRAM_STORAGE_PARTITION_SIZE \
						- DEVICE_MRAM_BASE_ADDR)
#else
/*
 * Regular non-MCUBoot app builds
 * Application uses entire flash area as executable.
 * Only storage partition needs device attribute for write access.
 */
#define FLASH_MRAM_BASE_ADDR	MRAM_BOOT_PARTITION_ADDR
#define FLASH_MRAM_SIZE		(MRAM_STORAGE_PARTITION_ADDR - MRAM_BOOT_PARTITION_ADDR)
/* Writable region: storage only */
#define DEVICE_MRAM_BASE_ADDR	MRAM_STORAGE_PARTITION_ADDR
#define DEVICE_MRAM_SIZE	MRAM_STORAGE_PARTITION_SIZE
#endif

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0: Executable MRAM */
	MPU_REGION_ENTRY("FLASH_MRAM", FLASH_MRAM_BASE_ADDR,
			 REGION_FLASH_ATTR(FLASH_MRAM_BASE_ADDR, FLASH_MRAM_SIZE)),
	/* Region 1: Writable MRAM (device mode for write access) */
	MPU_REGION_ENTRY("DEVICE_MRAM", DEVICE_MRAM_BASE_ADDR,
			 REGION_DEVICE_ATTR(DEVICE_MRAM_BASE_ADDR, DEVICE_MRAM_SIZE)),
	/* Region 2 */
	MPU_REGION_ENTRY("ITCM", DT_REG_ADDR(DT_NODELABEL(itcm)),
			 REGION_FLASH_ATTR(DT_REG_ADDR(DT_NODELABEL(itcm)),
							DT_REG_SIZE(DT_NODELABEL(itcm)))),
	/* Region 3 */
	MPU_REGION_ENTRY("DTCM", DT_REG_ADDR(DT_NODELABEL(dtcm)),
			 REGION_RAM_ATTR(DT_REG_ADDR(DT_NODELABEL(dtcm)),
							DT_REG_SIZE(DT_NODELABEL(dtcm)))),
	/* Region 4 */
	MPU_REGION_ENTRY("OSPI_CTRL", ALIF_HOST_OSPI_REG,
			 REGION_DEVICE_ATTR(ALIF_HOST_OSPI_REG, ALIF_HOST_OSPI_SIZE)),
	/* Region 5 */
	MPU_REGION_ENTRY("PERIPHERALS", ALIF_HOST_PERIPHERAL_BASE,
			 REGION_DEVICE_ATTR(ALIF_HOST_PERIPHERAL_BASE,
							ALIF_HOST_PERIPHERAL_SIZE)),
	/* Region 6 */
	MPU_REGION_ENTRY("OSPI0_XIP", ALIF_HOST_OSPI0_XIP_BASE,
			 REGION_OSPI_FLASH_ATTR(ALIF_HOST_OSPI0_XIP_BASE,
							ALIF_HOST_OSPI0_XIP_SIZE)),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
