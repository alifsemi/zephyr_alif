
/*
 * Copyright (C) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "feedback.h"

/* Nominal feedback values for 48 kHz audio. */
#define FEEDBACK_HS_NOMINAL (6U << 16)
#define FEEDBACK_FS_NOMINAL (48U << 14)

/* PI controller parameters. */
#define FEEDBACK_UPDATE_PERIOD  128U
#define FEEDBACK_MAX_CORRECTION 96
#define FEEDBACK_KP             12
#define FEEDBACK_KI_DIV         8
#define FEEDBACK_INTEGRAL_MAX   256

struct feedback_ctx {
	struct k_mem_slab *slab;

	uint32_t value;
	uint32_t sample_count;
	uint32_t used_sum;

	int32_t target_used;
	int32_t integral_sum;

	bool high_speed;
	bool running;
};

static struct feedback_ctx feedback_context = {
	.value = FEEDBACK_HS_NOMINAL,
	.high_speed = true,
};

static uint32_t feedback_nominal(const struct feedback_ctx *ctx)
{
	return ctx->high_speed ? FEEDBACK_HS_NOMINAL : FEEDBACK_FS_NOMINAL;
}

void feedback_reset_ctx(struct feedback_ctx *ctx)
{
	if (ctx == NULL) {
		return;
	}

	ctx->value = feedback_nominal(ctx);
	ctx->sample_count = 0U;
	ctx->used_sum = 0U;
	ctx->target_used = 0;
	ctx->integral_sum = 0;
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
	 * Use initial slab occupancy as the target.
	 * The slab includes USB and I2S buffers.
	 */
	ctx->target_used = (int32_t)k_mem_slab_num_used_get(ctx->slab);

	ctx->running = true;
}

void feedback_process(struct feedback_ctx *ctx)
{
	uint32_t used_sum;
	int32_t error_sum;
	int32_t integral_limit;
	int32_t correction;

	if (ctx == NULL || !ctx->running || ctx->slab == NULL) {
		return;
	}

	ctx->used_sum += k_mem_slab_num_used_get(ctx->slab);

	ctx->sample_count++;

	if (ctx->sample_count < FEEDBACK_UPDATE_PERIOD) {
		return;
	}

	used_sum = ctx->used_sum;

	error_sum = (int32_t)used_sum - ctx->target_used * (int32_t)FEEDBACK_UPDATE_PERIOD;

	ctx->used_sum = 0U;
	ctx->sample_count = 0U;

	integral_limit = FEEDBACK_INTEGRAL_MAX * (int32_t)FEEDBACK_UPDATE_PERIOD;

	ctx->integral_sum = CLAMP(ctx->integral_sum + error_sum, -integral_limit, integral_limit);

	/*
	 * Increase feedback when occupancy is low,
	 * decrease feedback when occupancy is high.
	 */
	correction = -((error_sum * FEEDBACK_KP) / (int32_t)FEEDBACK_UPDATE_PERIOD +
		       ctx->integral_sum / ((int32_t)FEEDBACK_UPDATE_PERIOD * FEEDBACK_KI_DIV));

	correction = CLAMP(correction, -FEEDBACK_MAX_CORRECTION, FEEDBACK_MAX_CORRECTION);

	if (ctx->high_speed) {
		ctx->value = (uint32_t)((int32_t)FEEDBACK_HS_NOMINAL + correction);
	} else {
		ctx->value = (uint32_t)((int32_t)FEEDBACK_FS_NOMINAL + correction * 2);
	}
}

uint32_t feedback_value(struct feedback_ctx *ctx)
{
	if (ctx == NULL) {
		return FEEDBACK_HS_NOMINAL;
	}

	return ctx->value;
}
