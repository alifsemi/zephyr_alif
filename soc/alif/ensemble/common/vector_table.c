/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/linker/linker-defs.h>

#define VECTOR_ADDRESS ((uintptr_t) _vector_start)

void relocate_vector_table(void)
{
	write_sctlr(read_sctlr() & ~HIVECS);
	write_vbar(VECTOR_ADDRESS & VBAR_MASK);
	barrier_isync_fence_full();
}
