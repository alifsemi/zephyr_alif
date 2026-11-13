/*
 * Copyright (C) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Occupancy feedback for implicit feedback. One feedback_process() call per
 * USB service interval is required: 125 us at High-Speed, 1 ms at Full-Speed.
 */

#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>

#include "feedback.h"

#define FEEDBACK_PERIOD_HS 256U
#define FEEDBACK_PERIOD_FS 32U

struct feedback_ctx {
	struct k_mem_slab *slab;
	uint32_t sample_count;
	uint32_t used_sum;
	int32_t target_used;
	int sample_offset;
	bool high_speed;
	bool running;
};

static struct feedback_ctx feedback_context;

void feedback_reset_ctx(struct feedback_ctx *ctx)
{
	if (ctx == NULL) {
		return;
	}

	ctx->sample_count = 0U;
	ctx->used_sum = 0U;
	ctx->target_used = 0;
	ctx->sample_offset = 0;
	ctx->running = false;
}

struct feedback_ctx *feedback_init(void)
{
	struct feedback_ctx *ctx = &feedback_context;

	ctx->slab = NULL;
	ctx->high_speed = true;
	feedback_reset_ctx(ctx);

	return ctx;
}

void feedback_bind_slab(struct feedback_ctx *ctx, struct k_mem_slab *slab)
{
	if (ctx == NULL) {
		return;
	}

	ctx->slab = slab;
}

void feedback_set_speed(struct feedback_ctx *ctx, bool high_speed)
{
	if (ctx == NULL) {
		return;
	}

	ctx->high_speed = high_speed;
	feedback_reset_ctx(ctx);
}

void feedback_start(struct feedback_ctx *ctx, int i2s_blocks_queued)
{
	ARG_UNUSED(i2s_blocks_queued);

	if (ctx == NULL) {
		return;
	}

	feedback_reset_ctx(ctx);

	if (ctx->slab == NULL) {
		printk("FB: slab not configured\n");
		return;
	}

	/*
	 * Use the slab occupancy at stream start as the target.
	 * The slab holds USB OUT buffers and I2S TX buffers.
	 */
	ctx->target_used = (int32_t)k_mem_slab_num_used_get(ctx->slab);
	ctx->running = true;
}

void feedback_process(struct feedback_ctx *ctx)
{
	uint32_t period;
	int32_t error;

	if (ctx == NULL || !ctx->running || ctx->slab == NULL) {
		return;
	}

	period = ctx->high_speed ? FEEDBACK_PERIOD_HS : FEEDBACK_PERIOD_FS;
	ctx->used_sum += k_mem_slab_num_used_get(ctx->slab);
	ctx->sample_count++;

	if (ctx->sample_count < period) {
		return;
	}

	error = (int32_t)ctx->used_sum - ctx->target_used * (int32_t)period;
	ctx->used_sum = 0U;
	ctx->sample_count = 0U;

	/* Leave an unconsumed correction in place. */
	if (ctx->sample_offset != 0) {
		return;
	}

	if (error >= (int32_t)period) {
		/* Buffer is filling. Ask the host to send fewer samples. */
		ctx->sample_offset = 1;
	} else if (error <= -(int32_t)period) {
		/* Buffer is draining. Ask the host to send more samples. */
		ctx->sample_offset = -1;
	}
}

int feedback_samples_offset(struct feedback_ctx *ctx)
{
	int offset;

	if (ctx == NULL) {
		return 0;
	}

	offset = ctx->sample_offset;
	ctx->sample_offset = 0;

	return offset;
}
