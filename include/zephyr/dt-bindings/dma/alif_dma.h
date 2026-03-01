/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_ALIF_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_ALIF_DMA_H_

/**
 * @brief Alif Event Router DMA configuration flags
 * @defgroup alif_dma Alif DMA Configuration
 * @ingroup devicetree
 * @{
 */

/**
 * @brief Enable hardware handshaking for DMA transfer
 *
 * This flag controls the DMA_ACK_TYPE register which enables hardware
 * handshaking between the peripheral and DMA controller. When enabled,
 * the DMA controller waits for peripheral acknowledgment before continuing
 * transfers, providing flow control.
 *
 * Typical usage:
 * - UART, SPI, I2S peripherals: Enable (flow control required)
 * - Memory-to-memory transfers: Disable (maximum speed)
 * - Other peripherals: Consult hardware manual
 *
 * This is used as the handshake parameter in ALIF_DMA_ENCODE().
 */
#define ALIF_DMA_HANDSHAKE_EN		0x00000001

/**
 * @brief Encode event router parameters into channel parameter
 *
 * Packs physical DMA channel, dma_group, and handshake into cell 0.
 * The peripheral request number is passed separately in cell 1.
 *
 * Bit allocation (channel in LSB):
 *   Bits 0-7: Physical DMA channel (0-255)
 *   Bits 8-11: dma_group - input source selection
 *              Current hardware: 0-3 (bits 8-9 used, bits 10-11 reserved)
 *              Encoding supports: 0-15 (for future expansion)
 *   Bit 12: handshake (0/1) - handshaking enable flag
 *   Bits 13-31: Reserved for future use (19 bits)
 *
 * Example:
 *   ALIF_DMA_ENCODE(2, 3, 1) = 0x1302
 *   Binary: 0001001100000010 = [hs=1][group=3][unused][channel=2]
 *
 * Device tree usage:
 *   dmas = <&evtrtr0 ALIF_DMA_ENCODE(0, 2, 1) 5>,   // ch=0, group=2, hs=on, periph=5
 *          <&evtrtr0 ALIF_DMA_ENCODE(1, 3, 1) 6>;   // ch=1, group=3, hs=on, periph=6
 *
 * @param channel Physical DMA channel (0-255)
 * @param dma_group Input source selection (current hardware: 0-3; encoding: 0-15)
 * @param handshake Handshaking enable (0 or 1, use ALIF_DMA_HANDSHAKE_EN)
 * @return Encoded value for DMA channel parameter
 */
#define ALIF_DMA_ENCODE(channel, dma_group, handshake) \
	(((handshake) << 12) | (((dma_group) & 0xF) << 8) | ((channel) & 0xFF))

/**
 * @brief Decode physical DMA channel from encoded value
 *
 * Extracts the physical DMA channel number from encoded channel parameter.
 *
 * @param encoded Encoded channel value
 * @return Physical DMA channel (0-255)
 */
#define ALIF_DMA_DECODE_CHANNEL(encoded) \
	((encoded) & 0xFF)

/**
 * @brief Decode dma_group from encoded value
 *
 * Extracts the input source selection from encoded channel parameter.
 *
 * @param encoded Encoded channel value
 * @return Input source selection (current hardware: 0-3; encoding: 0-15)
 */
#define ALIF_DMA_DECODE_GROUP(encoded) \
	(((encoded) >> 8) & 0xF)

/**
 * @brief Decode handshake flag from encoded value
 *
 * Extracts the handshaking enable flag from encoded channel parameter.
 *
 * @param encoded Encoded channel value
 * @return Handshaking enable flag (0 or 1)
 */
#define ALIF_DMA_DECODE_HANDSHAKE(encoded) \
	(((encoded) >> 12) & 0x1)

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_DMA_ALIF_DMA_H_ */
