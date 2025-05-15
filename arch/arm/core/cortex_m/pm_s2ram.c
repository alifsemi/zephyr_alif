/*
 * Copyright (c) 2022, Carlo Caione <ccaione@baylibre.com>
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/linker/sections.h>

#include <zephyr/arch/common/pm_s2ram.h>
#if defined (CONFIG_SOC_FAMILY_ENSEMBLE) || defined (CONFIG_SOC_FAMILY_BALLETTO)
#include "sau_tcm_ns_setup.h"
#endif
#include <cmsis_core.h>
#include <string.h>

/**
 * CPU context for S2RAM
 */
__noinit _cpu_context_t _cpu_context;

/**
 * S2RAM Marker
 */
__noinit uint32_t marker;

#define NVIC_MEMBER_SIZE(member) ARRAY_SIZE(((NVIC_Type *)0)->member)

/* Currently dynamic regions are only used in case of userspace or stack guard and
 * stack guard is not used by default on Cortex-M33 because there is a dedicated
 * mechanism for stack overflow detection. Unless those condition change we don't
 * need to store MPU content, it can just be reinitialized on resuming.
 */
#define MPU_USE_DYNAMIC_REGIONS IS_ENABLED(CONFIG_USERSPACE) || IS_ENABLED(CONFIG_MPU_STACK_GUARD)

/* TODO: The num-mpu-regions property should be used. Needs to be added to dts bindings. */
#define MPU_MAX_NUM_REGIONS 16

#define SAU_MAX_NUM_REGIONS 8
#define SAU_TYPE_NUM_REGION_Msk 0xF

typedef struct {
	/* NVIC components stored into RAM. */
	uint32_t ISER[NVIC_MEMBER_SIZE(ISER)];
	uint32_t ISPR[NVIC_MEMBER_SIZE(ISPR)];
	uint8_t IPR[NVIC_MEMBER_SIZE(IPR)];
} _nvic_context_t;

#if defined (MPU)
typedef struct {
	uint32_t RNR;
	uint32_t RBAR[MPU_MAX_NUM_REGIONS];
	uint32_t RLAR[MPU_MAX_NUM_REGIONS];
	uint32_t MAIR0;
	uint32_t MAIR1;
	uint32_t CTRL;
} _mpu_context_t;
#endif

typedef struct {
	uint32_t ICSR;
	uint32_t VTOR;
	uint32_t AIRCR;
	uint32_t SCR;
	uint32_t CCR;
	uint8_t SHPR[12U];
	uint32_t SHCSR;
	uint32_t CFSR;
	uint32_t HFSR;
	uint32_t DFSR;
	uint32_t MMFAR;
	uint32_t BFAR;
	uint32_t AFSR;
#if (defined (__FPU_USED) && (__FPU_USED == 1U)) || \
	(defined (__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
	uint32_t CPACR;
#endif
} _scb_context_t;

#if defined (ICB)
typedef struct {
	uint32_t ACTLR;
	uint32_t CPPWR;
} _icb_context_t;
#endif

#if defined (MEMSYSCTL)
typedef struct {
	uint32_t MSCR;
	uint32_t PFCR;
	uint32_t ITGU_CTRL;
	uint32_t ITGU_LUT[16];
	uint32_t DTGU_CTRL;
	uint32_t DTGU_LUT[16];
} _memsysctl_context_t;
#endif

#if defined (SAU)
typedef struct {
	uint32_t CTRL;
	uint32_t TYPE;
	uint32_t RBAR[SAU_MAX_NUM_REGIONS];
	uint32_t RLAR[SAU_MAX_NUM_REGIONS];
} _sau_context_t;
#endif

#if (defined(__FPU_USED) && (__FPU_USED == 1U)) || \
	(defined(__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))

/* We only need to preserve APCS callee-preserved registers */
typedef struct {
	double D[8];
	uint32_t FPSCR;
#if defined(__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U)
	uint32_t VPR;
#endif
	uint32_t FPCCR;
	uint32_t FPCAR;
	uint32_t FPDSCR;
	bool saved;
} _fpu_context_t;
#endif

struct backup {
	_nvic_context_t nvic_context;
#if defined (MPU)
	_mpu_context_t mpu_context;
#endif
	_scb_context_t scb_context;
#if defined (ICB)
	_icb_context_t icb_context;
#endif
#if defined (MEMSYSCTL)
	_memsysctl_context_t memsysctl_context;
#endif
#if defined (SAU)
	_sau_context_t sau_context;
#endif
#if (defined(__FPU_USED) && (__FPU_USED == 1U)) || \
	(defined(__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
	_fpu_context_t fpu_context;
#endif
};

static __noinit struct backup backup_data;

#if defined (MPU)
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
#endif

static void nvic_suspend(_nvic_context_t *backup)
{
	memcpy(backup->ISER, (const void *)&NVIC->ISER, sizeof(NVIC->ISER));
	memcpy(backup->ISPR, (const void *)&NVIC->ISPR, sizeof(NVIC->ISPR));
	memcpy(backup->IPR, (const void *)&NVIC->IPR, sizeof(NVIC->IPR));
}

static void nvic_resume(_nvic_context_t *backup)
{
	memcpy((void *)&NVIC->ISER, backup->ISER, sizeof(NVIC->ISER));
	memcpy((void *)&NVIC->ISPR, backup->ISPR, sizeof(NVIC->ISPR));
	memcpy((void *)&NVIC->IPR, backup->IPR, sizeof(NVIC->IPR));
}

static void scb_suspend(_scb_context_t *backup)
{
	backup->ICSR = SCB->ICSR;
	backup->VTOR = SCB->VTOR;
	backup->AIRCR = SCB->AIRCR;
	backup->SCR = SCB->SCR;
	backup->CCR = SCB->CCR;
	memcpy(backup->SHPR, (const void *)&SCB->SHPR, sizeof(SCB->SHPR));
	backup->SHCSR = SCB->SHCSR;
	backup->CFSR = SCB->CFSR;
	backup->HFSR = SCB->HFSR;
	backup->DFSR = SCB->DFSR;
	backup->MMFAR = SCB->MMFAR;
	backup->BFAR = SCB->BFAR;
	backup->AFSR = SCB->AFSR;
#if (defined (__FPU_USED) && (__FPU_USED == 1U)) || \
	(defined (__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
	backup->CPACR = SCB->CPACR;
#endif
}

static void scb_resume(_scb_context_t *backup)
{
	SCB->ICSR = backup->ICSR;
	SCB->VTOR = backup->VTOR;
	SCB->AIRCR = (backup->AIRCR & (~(SCB_AIRCR_VECTKEY_Msk)))
		| (((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) &
			SCB_AIRCR_VECTKEY_Msk);
	SCB->SCR = backup->SCR;
	SCB->CCR = backup->CCR;
	memcpy((void *)&SCB->SHPR, backup->SHPR, sizeof(SCB->SHPR));
	SCB->SHCSR = backup->SHCSR;
	SCB->CFSR = backup->CFSR;
	SCB->HFSR = backup->HFSR;
	SCB->DFSR = backup->DFSR;
	SCB->MMFAR = backup->MMFAR;
	SCB->BFAR = backup->BFAR;
	SCB->AFSR = backup->AFSR;
#if (defined (__FPU_USED) && (__FPU_USED == 1U)) || \
	(defined (__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
	SCB->CPACR = backup->CPACR;
#endif
	__DSB();
	__ISB();
}

#if defined (ICB)
static void icb_suspend(_icb_context_t *backup)
{
	backup->ACTLR = ICB->ACTLR;
	backup->CPPWR = ICB->CPPWR;
}

static void icb_resume(_icb_context_t *backup)
{
	ICB->ACTLR = backup->ACTLR;
	ICB->CPPWR = backup->CPPWR;
}
#endif

#if defined (MEMSYSCTL)
static void memsysctl_suspend(_memsysctl_context_t *backup)
{
	backup->MSCR = MEMSYSCTL->MSCR;
	backup->PFCR = MEMSYSCTL->PFCR;
	backup->ITGU_CTRL = MEMSYSCTL->ITGU_CTRL;
	memcpy(backup->ITGU_LUT, (const void *)&MEMSYSCTL->ITGU_LUT, sizeof(MEMSYSCTL->ITGU_LUT));
	backup->DTGU_CTRL = MEMSYSCTL->DTGU_CTRL;
	memcpy(backup->DTGU_LUT, (const void *)&MEMSYSCTL->DTGU_LUT, sizeof(MEMSYSCTL->DTGU_LUT));
}

static void memsysctl_resume(_memsysctl_context_t *backup)
{
	MEMSYSCTL->MSCR = backup->MSCR;
	MEMSYSCTL->PFCR = backup->PFCR;
	MEMSYSCTL->ITGU_CTRL = backup->ITGU_CTRL;
	memcpy((void *)&MEMSYSCTL->ITGU_LUT, backup->ITGU_LUT, sizeof(MEMSYSCTL->ITGU_LUT));
	MEMSYSCTL->DTGU_CTRL = backup->DTGU_CTRL;
	memcpy((void *)&MEMSYSCTL->DTGU_LUT, backup->DTGU_LUT, sizeof(MEMSYSCTL->DTGU_LUT));
}
#endif

#if defined (SAU)
static void sau_suspend(_sau_context_t *backup)
{
	uint8_t nr;
	backup->CTRL = SAU->CTRL;
	nr = SAU->TYPE & SAU_TYPE_NUM_REGION_Msk;
	for (uint8_t i = 0; i < nr; i++) {
		SAU->RNR = i;
		backup->RBAR[i] = SAU->RBAR;
		backup->RLAR[i] = SAU->RLAR;
	}
}

static void sau_resume(_sau_context_t *backup)
{
	uint8_t nr;
	nr = SAU->TYPE & SAU_TYPE_NUM_REGION_Msk;
	for (uint8_t i = 0; i < nr; i++) {
		SAU->RNR = i;
		SAU->RBAR = backup->RBAR[i];
		SAU->RLAR = backup->RLAR[i];
	}

	SAU->CTRL = backup->CTRL;
}
#endif

#if (defined(__FPU_USED) && (__FPU_USED == 1U)) || \
	(defined(__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
static inline void fpu_suspend(_fpu_context_t *backup)
{
	backup->saved = false;
	if (!(ICB->CPPWR & ICB_CPPWR_SU10_Msk)) {

		/* Only need to save if we have our own floating point context active,
		 * or lazy stack preservation is active, indicating registers hold
		 * another context's state.
		 */
		if ((__get_CONTROL() & CONTROL_FPCA_Msk) ||
			(FPU->FPCCR & FPU_FPCCR_LSPACT_Msk)) {
			__asm__ volatile(
				"VSTM %0, {D8-D15}\n\t"
				"VSTR FPSCR, [%0, #64]\n\t"
#if defined(__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U)
				"VSTR VPR, [%0, #68]\n\t"
#endif
				:
				: "r"(backup)
				: "memory");
			backup->saved = true;
		}

		backup->FPCCR = FPU->FPCCR;
		backup->FPCAR = FPU->FPCAR;
		backup->FPDSCR = FPU->FPDSCR;
	}

}

static inline void fpu_resume(_fpu_context_t *backup)
{
	FPU->FPCCR = backup->FPCCR;
	FPU->FPCAR = backup->FPCAR;
	FPU->FPDSCR = backup->FPDSCR;

	if (backup->saved) {
		__asm__ volatile(
			"VLDM %0, {D8-D15}\n\t"
			"VLDR FPSCR, [%0, #64]\n\t"
#if defined(__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U)
			"VLDR VPR, [%0, #68]\n\t"
#endif
			:
			: "r"(backup)
			: "memory");
	}
	backup->saved = false;
}
#endif

void pm_s2ram_save_ext_regs(void)
{
	/* Save context */
	scb_suspend(&backup_data.scb_context);
	nvic_suspend(&backup_data.nvic_context);
#if defined (MPU)
	mpu_suspend(&backup_data.mpu_context);
#endif
#if defined (ICB)
	icb_suspend(&backup_data.icb_context);
#endif
#if defined (MEMSYSCTL)
	memsysctl_suspend(&backup_data.memsysctl_context);
#endif
#if defined (SAU)
	sau_suspend(&backup_data.sau_context);
#endif
#if (defined(__FPU_USED) && (__FPU_USED == 1U)) || \
	(defined(__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
	fpu_suspend(&backup_data.fpu_context);
#endif
	__DMB();
}

void pm_s2ram_restore_ext_regs(void)
{
	/* Restore context */
#if defined (ICB)
	icb_resume(&backup_data.icb_context);
#endif
	scb_resume(&backup_data.scb_context);
#if (defined(__FPU_USED) && (__FPU_USED == 1U)) || \
	(defined(__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
	fpu_resume(&backup_data.fpu_context);
#endif
	nvic_resume(&backup_data.nvic_context);
#if defined (MEMSYSCTL)
	memsysctl_resume(&backup_data.memsysctl_context);
#endif
#if defined (SAU)
	sau_resume(&backup_data.sau_context);
#endif
#if defined (MPU)
	mpu_resume(&backup_data.mpu_context);
#endif
	__DSB();
	__ISB();
}
