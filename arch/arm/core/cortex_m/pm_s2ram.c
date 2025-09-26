/*
 * Copyright (c) 2022, Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/linker/sections.h>

#include <zephyr/arch/common/pm_s2ram.h>
#include <string.h>

#define MAGIC (0xDABBAD00)

#define NVIC_MEMBER_SIZE(member) ARRAY_SIZE(((NVIC_Type *)0)->member)

/* Currently dynamic regions are only used in case of userspace or stack guard and
 * stack guard is not used by default on Cortex-M33 because there is a dedicated
 * mechanism for stack overflow detection. Unless those condition change we don't
 * need to store MPU content, it can just be reinitialized on resuming.
 */
#define MPU_USE_DYNAMIC_REGIONS IS_ENABLED(CONFIG_USERSPACE) || IS_ENABLED(CONFIG_MPU_STACK_GUARD)

/* TODO: The num-mpu-regions property should be used. Needs to be added to dts bindings. */
#define MPU_MAX_NUM_REGIONS 16

typedef struct {
	/* NVIC components stored into RAM. */
	uint32_t ISER[NVIC_MEMBER_SIZE(ISER)];
	uint32_t ISPR[NVIC_MEMBER_SIZE(ISPR)];
	uint8_t IPR[NVIC_MEMBER_SIZE(IPR)];
} _nvic_context_t;

typedef struct {
	uint32_t RNR;
	uint32_t RBAR[MPU_MAX_NUM_REGIONS];
	uint32_t RLAR[MPU_MAX_NUM_REGIONS];
	uint32_t MAIR0;
	uint32_t MAIR1;
	uint32_t CTRL;
} _mpu_context_t;

struct backup {
	_nvic_context_t nvic_context;
	_mpu_context_t mpu_context;
};

/**
 * CPU context for S2RAM
 */
__noinit _cpu_context_t _cpu_context;

static __noinit struct backup backup_data;

extern void z_arm_configure_static_mpu_regions(void);
extern int z_arm_mpu_init(void);

/* MPU registers cannot be simply copied because content of RBARx RLARx registers
 * depends on region which is selected by RNR register.
 */
static void mpu_suspend(_mpu_context_t *backup)
{
	if (!MPU_USE_DYNAMIC_REGIONS) {
		return;
	}

	backup->RNR = MPU->RNR;

	for (uint8_t i = 0; i < MPU_MAX_NUM_REGIONS; i++) {
		MPU->RNR = i;
		backup->RBAR[i] = MPU->RBAR;
		backup->RLAR[i] = MPU->RLAR;
	}
	backup->MAIR0 = MPU->MAIR0;
	backup->MAIR1 = MPU->MAIR1;
	backup->CTRL = MPU->CTRL;
}

static void mpu_resume(_mpu_context_t *backup)
{
	if (!MPU_USE_DYNAMIC_REGIONS) {
		z_arm_mpu_init();
		z_arm_configure_static_mpu_regions();
		return;
	}

	uint32_t rnr = backup->RNR;

	for (uint8_t i = 0; i < MPU_MAX_NUM_REGIONS; i++) {
		MPU->RNR = i;
		MPU->RBAR = backup->RBAR[i];
		MPU->RLAR = backup->RLAR[i];
	}

	MPU->MAIR0 = backup->MAIR0;
	MPU->MAIR1 = backup->MAIR1;
	MPU->RNR = rnr;
	MPU->CTRL = backup->CTRL;
}

static void nvic_suspend(_nvic_context_t *backup)
{
	memcpy(backup->ISER, (uint32_t *)NVIC->ISER, sizeof(NVIC->ISER));
	memcpy(backup->ISPR, (uint32_t *)NVIC->ISPR, sizeof(NVIC->ISPR));
	memcpy(backup->IPR, (uint32_t *)NVIC->IPR, sizeof(NVIC->IPR));
}

static void nvic_resume(_nvic_context_t *backup)
{
	memcpy((uint32_t *)NVIC->ISER, backup->ISER, sizeof(NVIC->ISER));
	memcpy((uint32_t *)NVIC->ISPR, backup->ISPR, sizeof(NVIC->ISPR));
	memcpy((uint32_t *)NVIC->IPR, backup->IPR, sizeof(NVIC->IPR));
}

void pm_s2ram_save_ext_regs(void)
{
	/* Save context */
	nvic_suspend(&backup_data.nvic_context);
	mpu_suspend(&backup_data.mpu_context);
}

void pm_s2ram_restore_ext_regs(void)
{
	/* Restore context */
	nvic_resume(&backup_data.nvic_context);
	mpu_resume(&backup_data.mpu_context);
}

#ifndef CONFIG_PM_S2RAM_CUSTOM_MARKING
/**
 * S2RAM Marker
 */
static __noinit uint32_t marker;

void __attribute__((naked)) pm_s2ram_mark_set(void)
{
	__asm__ volatile(
		/* Set the marker to MAGIC value */
		"str	%[_magic_val], [%[_marker]]\n"

		"bx	lr\n"
		:
		: [_magic_val] "r"(MAGIC), [_marker] "r"(&marker)
		: "r1", "r4", "memory");
}

bool __attribute__((naked)) pm_s2ram_mark_check_and_clear(void)
{
	__asm__ volatile(
		/* Set return value to 0 */
		"mov	r0, #0\n"

		/* Check the marker */
		"ldr	r3, [%[_marker]]\n"
		"cmp	r3, %[_magic_val]\n"
		"bne	exit\n"

		/*
		 * Reset the marker
		 */
		"str	r0, [%[_marker]]\n"

		/*
		 * Set return value to 1
		 */
		"mov	r0, #1\n"

		"exit:\n"
		"bx lr\n"
		:
		: [_magic_val] "r"(MAGIC), [_marker] "r"(&marker)
		: "r0", "r1", "r3", "r4", "memory");
}

#endif /* CONFIG_PM_S2RAM_CUSTOM_MARKING */
