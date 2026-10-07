/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FEEDBACK_H_
#define FEEDBACK_H_

#include <stdint.h>
#include <zephyr/kernel.h>

/* Nominal number of samples received on each SOF. This sample is currently
 * supporting only 48 kHz sample rate.
 */
#define SAMPLES_PER_SOF     48

struct feedback_ctx *feedback_init(void);
void feedback_reset_ctx(struct feedback_ctx *ctx);
void feedback_process(struct feedback_ctx *ctx);
void feedback_start(struct feedback_ctx *ctx, int i2s_blocks_queued);
uint32_t feedback_value(struct feedback_ctx *ctx);
#if defined(CONFIG_SOC_FAMILY_BALLETTO) || \
	defined(CONFIG_SOC_FAMILY_ENSEMBLE)
void feedback_bind_slab(struct feedback_ctx *ctx,
			struct k_mem_slab *slab);
void feedback_set_speed(struct feedback_ctx *ctx,
			bool high_speed);
#endif

#endif /* FEEDBACK_H_ */
