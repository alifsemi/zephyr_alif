/*
 * Copyright (c) 2019, Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>

#include <zephyr/drivers/video.h>

#if defined(CONFIG_SOC_FAMILY_ENSEMBLE) && !defined(CONFIG_SOC_SERIES_ENSEMBLE_E1C)
#define VIDEO_HEAP_SIZE (CONFIG_VIDEO_BUFFER_POOL_SZ_MAX * CONFIG_VIDEO_BUFFER_POOL_NUM_MAX)
static struct k_heap video_buffer_pool;
static uint8_t video_buff[VIDEO_HEAP_SIZE] __aligned(8) __attribute__((section(".video_buffer")));
#else
K_HEAP_DEFINE(video_buffer_pool, CONFIG_VIDEO_BUFFER_POOL_SZ_MAX *
		CONFIG_VIDEO_BUFFER_POOL_NUM_MAX);
#endif

static struct video_buffer video_buf[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];

struct mem_block {
	void *data;
};

static struct mem_block video_block[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];

struct video_buffer *video_buffer_alloc(size_t size)
{
	struct video_buffer *vbuf = NULL;
	struct mem_block *block;
	int i;

	/* find available video buffer */
	for (i = 0; i < ARRAY_SIZE(video_buf); i++) {
		if (video_buf[i].buffer == NULL) {
			vbuf = &video_buf[i];
			block = &video_block[i];
			break;
		}
	}

	if (vbuf == NULL) {
		return NULL;
	}

	/* Alloc buffer memory */
	block->data = k_heap_alloc(&video_buffer_pool, size, K_FOREVER);
	if (block->data == NULL) {
		return NULL;
	}

	vbuf->buffer = block->data;
	vbuf->size = size;
	vbuf->bytesused = 0;

	return vbuf;
}

void video_buffer_release(struct video_buffer *vbuf)
{
	struct mem_block *block = NULL;
	int i;

	/* vbuf to block */
	for (i = 0; i < ARRAY_SIZE(video_block); i++) {
		if (video_block[i].data == vbuf->buffer) {
			block = &video_block[i];
			break;
		}
	}

	vbuf->buffer = NULL;

	if (block) {
		k_heap_free(&video_buffer_pool, block->data);
	}
}

#if defined(CONFIG_SOC_FAMILY_ENSEMBLE) && !defined(CONFIG_SOC_SERIES_ENSEMBLE_E1C)
static int heap_initialize(void)
{
	k_heap_init(&video_buffer_pool, video_buff, sizeof(video_buff));
	return 0;
}

SYS_INIT(heap_initialize, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif
