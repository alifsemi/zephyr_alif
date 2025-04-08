/*
 * Copyright (c) 2024 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cmsis_core.h>

void SystemInit (void);
extern void *_vector_table[];

#ifdef CONFIG_PLATFORM_SPECIFIC_INIT
__attribute__((naked)) void z_arm_platform_init(void)
{
	/* Needs to implement SOC specific SystemInit API */
	__asm(" LDR R0, =_vector_table 	\n");
	__asm(" LDR R1, [R0, 0] 		\n");
	__asm(" MSR MSP, R1 			\n");
	__asm(" PUSH {LR} 				\n");

	SystemInit();

	__asm(" POP {PC} \n");

}
#endif /* CONFIG_PLATFORM_SPECIFIC_INIT */
