/*
 * Copyright (c) 2025 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2S_I2S_DW_H_
#define ZEPHYR_DRIVERS_I2S_I2S_DW_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* Number of bytes per sample for 16/32-bit resolution */
#define I2S_DW_16BIT_SAMPLE_BYTES	2
#define I2S_DW_32BIT_SAMPLE_BYTES	4

/* FIFO depth for Tx & Rx */
#define I2S_DW_FIFO_DEPTH		16

/* Register offsets */
#define I2S_DW_IER			0x000 /* Global Enable */
#define I2S_DW_IRER			0x004 /* Rx Block Enable */
#define I2S_DW_ITER			0x008 /* Tx Block Enable */
#define I2S_DW_CER			0x00C /* Clock Enable */
#define I2S_DW_CCR			0x010 /* Clock Configuration */
#define I2S_DW_RXFFR			0x014 /* Rx Block FIFO Reset */
#define I2S_DW_TXFFR			0x018 /* Tx Block FIFO Reset */
#define I2S_DW_LRBR			0x020 /* Left Rx Buffer / Left Tx Holding */
#define I2S_DW_RRBR			0x024 /* Right Rx Buffer / Right Tx Holding */
#define I2S_DW_RER			0x028 /* Rx Channel Enable */
#define I2S_DW_TER			0x02C /* Tx Channel Enable */
#define I2S_DW_RCR			0x030 /* Rx Configuration */
#define I2S_DW_TCR			0x034 /* Tx Configuration */
#define I2S_DW_ISR			0x038 /* Interrupt Status */
#define I2S_DW_IMR			0x03C /* Interrupt Mask */
#define I2S_DW_ROR			0x040 /* Rx Overrun */
#define I2S_DW_TOR			0x044 /* Tx Overrun */
#define I2S_DW_RFCR			0x048 /* Rx FIFO Configuration */
#define I2S_DW_TFCR			0x04C /* Tx FIFO Configuration */
#define I2S_DW_RFF			0x050 /* Rx Channel FIFO Reset */
#define I2S_DW_TFF			0x054 /* Tx Channel FIFO Reset */
#define I2S_DW_RXDMA			0x1C0 /* Rx Block DMA */
#define I2S_DW_TXDMA			0x1C8 /* Tx Block DMA */
#define I2S_DW_DMACR			0x200 /* DMA Control */

/* Register bitfields */

/* IER */
#define I2S_DW_IER_IEN			BIT(0)

/* IRER */
#define I2S_DW_IRER_RXEN		BIT(0)

/* ITER */
#define I2S_DW_ITER_TXEN		BIT(0)

/* CER */
#define I2S_DW_CER_CLKEN		BIT(0)

/* CCR */
#define I2S_DW_CCR_SCLKG_MASK		GENMASK(2, 0)
#define I2S_DW_CCR_WSS_MASK		GENMASK(4, 3)
#define I2S_DW_CCR_WSS_SHIFT		3

/* RER / TER */
#define I2S_DW_RER_RXCHEN		BIT(0)
#define I2S_DW_TER_TXCHEN		BIT(0)

/* RCR / TCR - Word Length */
#define I2S_DW_WLEN_MASK		GENMASK(2, 0)

/* ISR */
#define I2S_DW_ISR_RXDA			BIT(0)
#define I2S_DW_ISR_RXFO			BIT(1)
#define I2S_DW_ISR_TXFE			BIT(4)
#define I2S_DW_ISR_TXFO			BIT(5)

/* IMR */
#define I2S_DW_IMR_RXDAM		BIT(0)
#define I2S_DW_IMR_RXFOM		BIT(1)
#define I2S_DW_IMR_TXFEM		BIT(4)
#define I2S_DW_IMR_TXFOM		BIT(5)

/* RFCR / TFCR - FIFO Trigger Level */
#define I2S_DW_FIFO_TRG_MASK		GENMASK(3, 0)

/* RFF / TFF - Channel FIFO Reset */
#define I2S_DW_FIFO_RST			BIT(0)

/* DMACR */
#define I2S_DW_DMACR_RXEN		BIT(16)
#define I2S_DW_DMACR_TXEN		BIT(17)

/* Hardware enumerations */

/* Word Select Size (number of SCLK cycles per channel) */
enum i2s_dw_wss {
	I2S_DW_WSS_16_CYCLES = 0,
	I2S_DW_WSS_24_CYCLES = 1,
	I2S_DW_WSS_32_CYCLES = 2,
};

/* SCLK Gating */
enum i2s_dw_sclkg {
	I2S_DW_SCLKG_NONE = 0,
	I2S_DW_SCLKG_12_CYCLES = 1,
	I2S_DW_SCLKG_16_CYCLES = 2,
	I2S_DW_SCLKG_20_CYCLES = 3,
	I2S_DW_SCLKG_24_CYCLES = 4,
};

/* Data Resolution (WLEN field encoding) */
enum i2s_dw_wlen {
	I2S_DW_WLEN_IGNORE = 0,
	I2S_DW_WLEN_12BIT  = 1,
	I2S_DW_WLEN_16BIT  = 2,
	I2S_DW_WLEN_20BIT  = 3,
	I2S_DW_WLEN_24BIT  = 4,
	I2S_DW_WLEN_32BIT  = 5,
};

/* Driver structures */
struct i2s_dw_stream {
	int32_t state;
	struct k_sem sem;
	struct k_msgq msgq;
	struct i2s_config cfg;
	void *mem_block;
	uint32_t mem_block_size;
	uint32_t mem_block_offset;
	bool last_block;
	bool master;
};

struct i2s_dw_queue_item {
	void *mem_block;
	size_t size;
};

/* Device constant configuration */
struct i2s_dw_cfg {
	DEVICE_MMIO_ROM;
	enum i2s_dw_wss wss_len;
	enum i2s_dw_sclkg sclkg;
	uint8_t tx_fifo_trg_lvl;
	uint8_t rx_fifo_trg_lvl;
	void (*irq_config)(const struct device *dev);
	const struct device *clk_dev;
	clock_control_subsys_t clkid;
#if defined(CONFIG_PINCTRL)
	const struct pinctrl_dev_config *pincfg;
#endif
};

/* Device runtime data */
struct i2s_dw_data {
	DEVICE_MMIO_RAM;
	enum i2s_dir dir;
	uint32_t imr_cache;
	struct i2s_dw_stream rx;
	struct i2s_dw_stream tx;
};

#endif /* ZEPHYR_DRIVERS_I2S_I2S_DW_H_ */
