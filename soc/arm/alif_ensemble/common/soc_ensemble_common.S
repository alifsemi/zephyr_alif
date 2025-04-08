/*
 * Copyright (c) 2025 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Platform specific init module for Alif Semiconductor ensemble family
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>

_ASM_FILE_PROLOGUE

GTEXT(SystemInit)
GTEXT(z_arm_platform_init)
GDATA(_vector_table)

#ifdef CONFIG_PLATFORM_SPECIFIC_INIT

SECTION_FUNC(TEXT, z_arm_platform_init)

	ldr r0, =_vector_table
	ldr r1, [r0, 0]
	msr msp, r1
	push {lr}
	bl SystemInit
	pop {pc}

#endif /* CONFIG_PLATFORM_SPECIFIC_INIT */
