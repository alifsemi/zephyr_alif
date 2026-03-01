/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Alif Event Router Control API
 *
 * This API provides direct control over the Alif Event Router channels
 * for non-DMA use cases. The Event Router can be used to route events
 * from one peripheral to another without involving DMA transfers.
 *
 * For DMA transfers, use the standard Zephyr DMA API with event router
 * device tree bindings instead of this control API.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_EVENT_ROUTER_ALIF_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_EVENT_ROUTER_ALIF_H_

#include <zephyr/device.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure an event router channel without DMA transfer
 *
 * This function enables or disables an event router channel for non-DMA
 * use cases. It configures only the event router hardware without setting
 * up a DMA transfer.
 *
 * @param dev Pointer to the event router device
 * @param channel Event router channel number (0-31)
 * @param dma_group Input source selection (0-3), selects which of 4
 *                  peripheral inputs feeds this channel
 * @param enable true to enable the channel, false to disable
 *
 * @retval 0 on success
 * @retval -EINVAL if channel or dma_group is out of range
 * @retval -ENODEV if device is not ready
 *
 * @note This function is thread-safe
 *
 * @warning Do not use this function for channels that are actively
 *          being used by the DMA subsystem. Use the DMA API for DMA
 *          transfers.
* @note This API only manages DMA_CTRL register, not DMA_ACK_TYPE (handshake).
 *       This is appropriate because channel usage (DMA vs non-DMA) is statically
 *       determined at compile time via device tree and does not change at runtime.
 *       DMA channels use the standard DMA API which manages handshake separately.
 *
 * Example usage for UTIMER loopback:
 * @code{.c}
 *     const struct device *evtrtr = DEVICE_DT_GET(DT_NODELABEL(evtrtr0));
 *
 *     // Enable event router channel 15 with dma_group 2
 *     // to route UTIMER output back to UTIMER input
 *     ret = alif_evtrtr_configure_channel(evtrtr, 15, 2, true);
 *     if (ret < 0) {
 *         printk("Failed to configure event router\n");
 *     }
 * @endcode
 */
int alif_evtrtr_configure_channel(const struct device *dev,
				   uint32_t channel,
				   uint32_t dma_group,
				   bool enable);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_EVENT_ROUTER_ALIF_H_ */
