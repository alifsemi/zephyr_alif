/*
 * Copyright (c) 2025 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Reset hook module for Alif Semiconductor ensemble family
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>

_ASM_FILE_PROLOGUE

GTEXT(SystemInit)
GTEXT(soc_reset_hook)
GDATA(_vector_table)

#ifdef CONFIG_SOC_RESET_HOOK

SECTION_FUNC(TEXT, soc_reset_hook)

	/* Reset CONTROL register */
	movs.n r0, #0
	msr CONTROL, r0
	isb

	/* Clear SPLIM registers */
	movs.n r0, #0
	msr MSPLIM, r0
	msr PSPLIM, r0

	/* Load the MSP value from the vector table
	 * and jump to SystemInit
	 */
	ldr r0, =_vector_table
	ldr r1, [r0, 0]
	msr msp, r1
	push {lr}
	bl SystemInit
	pop {pc}

#endif /* CONFIG_SOC_RESET_HOOK*/
