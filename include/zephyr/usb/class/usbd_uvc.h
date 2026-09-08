/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 * Copyright (C) 2026 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Adapted from the upstream Zephyr v4.2 USB Video Class for a Zephyr revision
 * whose video subsystem still uses the endpoint-based (enum video_endpoint_id)
 * API. The public interface is unchanged from upstream.
 */

/**
 * @file
 * @brief USB Video Class (UVC) public header
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H
#define ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H

#include <zephyr/device.h>

/**
 * @brief USB Video Class (UVC) device API
 * @defgroup usbd_uvc USB Video Class (UVC) device API
 * @ingroup usb
 * @since 4.2
 * @version 0.1.0
 * @see uvc: "Universal Serial Bus Device Class Definition for Video Devices"
 *      Document Release 1.5 (August 9, 2012)
 * @{
 */

/**
 * @brief Set the video device that a UVC instance will use.
 *
 * It will query its supported formats and frame rates, and use this information
 * to generate USB descriptors sent to the host.
 *
 * @note This function must be called before @ref usbd_enable.
 *
 * @param uvc_dev The UVC device
 * @param video_dev The video device that this UVC instance streams from
 */
void uvc_set_video_dev(const struct device *uvc_dev, const struct device *video_dev);

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H */
