/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_H_
#define _SOC_H_

#include <zephyr/arch/arm/cortex_m/nvic.h>

#define __MPU_PRESENT             1U        /* MPU regions present */
#define __SAUREGION_PRESENT       1U        /* SAU regions present */
#define __FPU_PRESENT             CONFIG_CPU_HAS_FPU
#define __DSP_PRESENT             1U        /* DSP extension present */
#define __MVE_PRESENT             1U        /* MVE extensions present */
#define __MVE_FP                  1U        /* MVE floating point present */
#define __ICACHE_PRESENT          1U        /* ICACHE present */
#define __DCACHE_PRESENT          1U        /* DCACHE present */

#define __NVIC_PRIO_BITS          NUM_IRQ_PRIO_BITS

typedef enum IRQn {
/* ---------------  Processor Exceptions Numbers  ------------------------- */
	Reset_IRQn                 = -15,
	NonMaskableInt_IRQn        = -14, /*  2 Non Maskable Interrupt */
	HardFault_IRQn             = -13, /*  3 HardFault Interrupt */
	MemoryManagement_IRQn      = -12, /*  4 Memory Management Interrupt */
	BusFault_IRQn              = -11, /*  5 Bus Fault Interrupt */
	UsageFault_IRQn            = -10, /*  6 Usage Fault Interrupt */
	SecureFault_IRQn           =  -9, /*  7 Secure Fault Interrupt */
	SVCall_IRQn                =  -5, /* 11 SV Call Interrupt */
	DebugMonitor_IRQn          =  -4, /* 12 Debug Monitor Interrupt */
	PendSV_IRQn                =  -2, /* 14 Pend SV Interrupt */
	SysTick_IRQn               =  -1, /* 15 System Tick Interrupt */
	Max_IRQn                   =  CONFIG_NUM_IRQS,
} IRQn_Type;

#define __Vendor_SysTickConfig         0 /* Default to standard SysTick */

#include <core_cm55.h>

#endif /* _SOC_H_ */
