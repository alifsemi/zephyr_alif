/**
 * @file
 *
 * @brief APIs for HM0360 camera sensor
 */

/*
 * Copyright (c) 2026 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <stddef.h>
#include <zephyr/kernel.h>

#include <zephyr/types.h>
#include <zephyr/drivers/video-controls.h>

#ifndef __ZEPHYR_INCLUDE_DRIVERS_HM0360_VIDEO_CONTROLS_H_
#define __ZEPHYR_INCLUDE_DRIVERS_HM0360_VIDEO_CONTROLS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define VIDEO_HM0360_CID_SNAPSHOT_CAPTURE	(VIDEO_CID_PRIVATE_BASE + 0x10)
#define VIDEO_HM0360_CID_CONTINUOUS_CAPTURE	(VIDEO_CID_PRIVATE_BASE + 0x11)

#define VIDEO_CID_SNAPSHOT_CAPTURE	VIDEO_HM0360_CID_SNAPSHOT_CAPTURE
#define VIDEO_CID_CONTINUOUS_CAPTURE	VIDEO_HM0360_CID_CONTINUOUS_CAPTURE

#ifdef __cplusplus
}
#endif

#endif /* __ZEPHYR_INCLUDE_DRIVERS_HM0360_VIDEO_CONTROLS_H_ */
