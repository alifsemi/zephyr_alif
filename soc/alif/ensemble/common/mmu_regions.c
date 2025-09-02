/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <stddef.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>
#include <zephyr/devicetree.h>
#include <zephyr/linker/linker-defs.h>

static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("vectors",
		(uintptr_t) _vector_start,
		0x100,
		MT_NORMAL | MPERM_R | MPERM_X | MATTR_SHARED |
		MATTR_CACHE_OUTER_WB_nWA | MATTR_CACHE_INNER_WB_nWA |
		MATTR_MAY_MAP_L1_SECTION),

	MMU_REGION_FLAT_ENTRY("GIC",
		DT_REG_ADDR_BY_IDX(DT_INST(0, arm_gic), 0),
		DT_REG_SIZE_BY_IDX(DT_INST(0, arm_gic), 0),
		MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),

	MMU_REGION_FLAT_ENTRY("GIC",
		DT_REG_ADDR_BY_IDX(DT_INST(0, arm_gic), 1),
		DT_REG_SIZE_BY_IDX(DT_INST(0, arm_gic), 1),
		MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),

	MMU_REGION_FLAT_ENTRY("UART",
		DT_REG_ADDR(DT_INST(0, ns16550)),
		DT_REG_SIZE(DT_INST(0, ns16550)),
		MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),

	/* CLKCTL_PER_MST & CLKCTL_PER_SLV regions */
	MMU_REGION_FLAT_ENTRY("CLKCTL",
		0x4902F000,
		0x11000,
		MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),

	/* PINMUX, VBAT, AON regions */
	MMU_REGION_FLAT_ENTRY("HOST PERIPHERALS",
		0x1A603000,
		0x7000,
		MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
