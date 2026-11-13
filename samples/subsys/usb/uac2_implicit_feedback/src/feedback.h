/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FEEDBACK_H_
#define FEEDBACK_H_

#include <stdbool.h>
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
#if defined(CONFIG_SOC_FAMILY_BALLETTO) || \
	defined(CONFIG_SOC_FAMILY_ENSEMBLE)
void feedback_bind_slab(struct feedback_ctx *ctx, struct k_mem_slab *slab);
void feedback_set_speed(struct feedback_ctx *ctx, bool high_speed);
#endif

/* Return offset between I2S block start and USB SOF in samples.
 *
 * Positive offset means that I2S block started at least 1 sample after SOF and
 * to correct the situation, shorter than nominal buffers are needed.
 *
 * Negative offset means that I2S block started at least 1 sample before SOF and
 * to correct the situation, larger than nominal buffers are needed.
 *
 * Offset 0 means that I2S block started within 1 sample around SOF. This is the
 * dominant value expected during normal operation.
 */
int feedback_samples_offset(struct feedback_ctx *ctx);

#endif /* FEEDBACK_H_ */
