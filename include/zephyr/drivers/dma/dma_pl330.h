/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM PL330 DMA Controller - External Microcode API
 *
 * This API allows applications to supply pre-built PL330 microcode directly,
 * bypassing the driver's internal microcode generation. The standard Zephyr
 * DMA API (dma_config) must still be called first to configure the channel
 * (callback, peripheral slot, direction).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_PL330_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_PL330_H_

#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start a DMA transfer using application-supplied PL330 microcode.
 *
 * Copies the provided microcode into the driver's channel exec buffer and
 * triggers execution. The standard dma_config() must be called before this
 * function to configure the channel callback, peripheral slot, and direction.
 *
 * @param dev        DMA device instance.
 * @param channel    Channel index. Must be configured via dma_config() first.
 * @param mcode_addr Pointer to the microcode buffer.
 * @param mcode_len  Length of the microcode in bytes.
 *                   Must not exceed MICROCODE_SIZE_MAX (default 1024 bytes).
 *
 * @retval 0          Success.
 * @retval -EINVAL    Invalid channel index or mcode_len > MICROCODE_SIZE_MAX.
 * @retval -EBUSY     Channel is already active.
 * @retval -ETIMEDOUT Polling-mode wait timed out (no callback configured).
 *
 * @note The event ID for DMASEV in custom microcode equals the channel number.
 * @note For peripheral transfers (M2P/P2M), FLUSHP/WFP/LDP/STP instructions
 *       must use the peripheral slot number passed to dma_config() via dma_slot.
 * @note The microcode is copied into the driver's internal buffer. The caller
 *       may free or reuse mcode_addr after this function returns.
 */
int dma_pl330_start_with_mcode(const struct device *dev,
				uint32_t channel,
				const uint8_t *mcode_addr,
				size_t mcode_len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_PL330_H_ */
