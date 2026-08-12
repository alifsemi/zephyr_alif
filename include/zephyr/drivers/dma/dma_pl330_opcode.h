/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM PL330 DMA Controller - Opcode Generation Helpers
 *
 * Provides inline helpers (dma_pl330_gen_*) for constructing PL330
 * microcode programs. This header has no Zephyr kernel dependencies
 * and can be used from both applications and kernel drivers.
 *
 * Typical usage:
 * @code{.c}
 *   uint8_t buf[MCODE_BUF_SIZE];
 *   struct dma_pl330_opcode_buf ob = { .buf = buf, .off = 0, .buf_size = sizeof(buf) };
 *   union dma_pl330_ccr ccr = { .value = 0 };
 *
 *   ccr.b.src_inc = ccr.b.dst_inc = 1;
 *   ccr.b.src_burst_size = ccr.b.dst_burst_size = 0;
 *   ccr.b.src_burst_len  = ccr.b.dst_burst_len  = 15;
 *
 *   dma_pl330_gen_move(ccr.value, DMA_PL330_REG_CCR, &ob);
 *   dma_pl330_gen_move((uint32_t)src, DMA_PL330_REG_SAR, &ob);
 *   dma_pl330_gen_move((uint32_t)dst, DMA_PL330_REG_DAR, &ob);
 *   dma_pl330_gen_loop(DMA_PL330_LC_0, n_bursts, &ob);
 *   uint32_t lp_start = ob.off;
 *   dma_pl330_gen_load(DMA_PL330_XFER_FORCE, &ob);
 *   dma_pl330_gen_store(DMA_PL330_XFER_FORCE, &ob);
 *   struct dma_pl330_loop lp = { DMA_PL330_XFER_FORCE, DMA_PL330_LC_0,
 *                                ob.off - lp_start, true };
 *   dma_pl330_gen_loopend(&lp, &ob);
 *   dma_pl330_gen_wmb(&ob);
 *   dma_pl330_gen_send_event(channel, &ob);
 *   dma_pl330_gen_end(&ob);
 * @endcode
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_PL330_OPCODE_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_PL330_OPCODE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opcode buffer tracker.
 *
 * Passed to all dma_pl330_gen_*() helpers. The helpers write encoded
 * instruction bytes sequentially into buf[] starting at offset off,
 * advancing off by the instruction size on each successful call.
 */
struct dma_pl330_opcode_buf {
	uint8_t  *buf;      /**< Start of the microcode buffer */
	uint32_t  off;      /**< Current write offset in bytes */
	uint32_t  buf_size; /**< Total buffer capacity in bytes */
};

/** DMAMOV destination register selector */
enum dma_pl330_reg {
	DMA_PL330_REG_SAR = 0, /**< Source Address Register */
	DMA_PL330_REG_CCR = 1, /**< Channel Control Register */
	DMA_PL330_REG_DAR = 2, /**< Destination Address Register */
};

/** Loop counter register selector */
enum dma_pl330_lc {
	DMA_PL330_LC_0 = 0, /**< Loop Counter 0 (inner loop) */
	DMA_PL330_LC_1 = 1, /**< Loop Counter 1 (outer loop) */
};

/**
 * @brief Transfer request type.
 *
 * Used to select the request mode for load/store and peripheral
 * instructions. Use DMA_PL330_XFER_FORCE for M2M transfers.
 */
enum dma_pl330_xfer {
	DMA_PL330_XFER_FORCE  = 0, /**< Force (DMALD/DMAST, no peripheral req) */
	DMA_PL330_XFER_SINGLE = 1, /**< Single request (DMALDS/DMASTS, DMAWFP-S) */
	DMA_PL330_XFER_BURST  = 2, /**< Burst request  (DMALDB/DMASTB, DMAWFP-B) */
	DMA_PL330_XFER_PERIPH = 3, /**< Peripheral     (DMAWFP-P only) */
};

/** Endian swap size for CCR @c endian_swap field */
enum dma_pl330_swap {
	DMA_PL330_SWAP_NONE   = 0, /**< No swap (8-bit data)            */
	DMA_PL330_SWAP_16BIT  = 1, /**< Swap bytes within 16-bit words  */
	DMA_PL330_SWAP_32BIT  = 2, /**< Swap bytes within 32-bit words  */
	DMA_PL330_SWAP_64BIT  = 3, /**< Swap bytes within 64-bit words  */
	DMA_PL330_SWAP_128BIT = 4, /**< Swap bytes within 128-bit words */
};

/**
 * @brief PL330 Channel Control Register (CCR) bitfield layout.
 *
 * Use ccr.b.* to set individual fields, then pass ccr.value to
 * dma_pl330_gen_move(..., DMA_PL330_REG_CCR, ...).
 */
union dma_pl330_ccr {
	uint32_t value;
	struct {
		uint32_t src_inc        : 1;  /**< Source address increment */
		uint32_t src_burst_size : 3;  /**< log2(bytes per beat): 0=1B..7=128B */
		uint32_t src_burst_len  : 4;  /**< Beats per burst minus 1 (0..15) */
		uint32_t src_prot_ctrl  : 3;  /**< AXI protection control (source) */
		uint32_t src_cache_ctrl : 3;  /**< AXI cache control (source) */
		uint32_t dst_inc        : 1;  /**< Destination address increment */
		uint32_t dst_burst_size : 3;  /**< log2(bytes per beat): 0=1B..7=128B */
		uint32_t dst_burst_len  : 4;  /**< Beats per burst minus 1 (0..15) */
		uint32_t dst_prot_ctrl  : 3;  /**< AXI protection control (destination) */
		uint32_t dst_cache_ctrl : 3;  /**< AXI cache control (destination) */
		uint32_t endian_swap    : 3;  /**< Endian swap size, see enum dma_pl330_swap */
	} b;
};

/**
 * @brief Loop-end instruction arguments for dma_pl330_gen_loopend().
 */
struct dma_pl330_loop {
	enum dma_pl330_xfer xfer_type; /**< Transfer type (FORCE/SINGLE/BURST) */
	enum dma_pl330_lc   lc;        /**< Loop counter register (LC0/LC1) */
	uint8_t             jump;      /**< Backward jump offset in bytes */
	bool                nf;        /**< true = count loop, false = loop-forever */
};

/* -----------------------------------------------------------------------
 * Opcode generation helpers
 * All helpers return true on success, false if the instruction does not
 * fit in the remaining buffer space (buf->off + instr_size > buf->buf_size).
 * -----------------------------------------------------------------------
 */

/**
 * @brief Emit DMAEND — marks end of microcode program.
 * Encoding: 1 byte (0x00).
 *
 * @param buf Opcode buffer.
 * @return    true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_end(struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 1U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x00U;
	return true;
}

/**
 * @brief Emit DMAMOV — load 32-bit immediate into SAR, CCR, or DAR.
 * Encoding: 6 bytes (opcode + reg + imm32 LE).
 *
 * @param imm  32-bit value to load (address or CCR value).
 * @param reg  Target register: SAR, CCR, or DAR.
 * @param buf  Opcode buffer.
 * @return     true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_move(uint32_t imm, enum dma_pl330_reg reg,
				      struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 6U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0xBCU;
	buf->buf[buf->off++] = (uint8_t)reg & 0x7U;
	buf->buf[buf->off++] = (uint8_t)(imm);
	buf->buf[buf->off++] = (uint8_t)(imm >> 8);
	buf->buf[buf->off++] = (uint8_t)(imm >> 16);
	buf->buf[buf->off++] = (uint8_t)(imm >> 24);
	return true;
}

/**
 * @brief Emit DMALD / DMALDS / DMALDB — DMA load from source.
 * Encoding: 1 byte.
 *
 * @param xfer  FORCE: always load; SINGLE: single req; BURST: burst req.
 * @param buf   Opcode buffer.
 * @return	    true if encoded; false if xfer is invalid or buffer is full.
 */
static inline bool dma_pl330_gen_load(enum dma_pl330_xfer xfer,
				      struct dma_pl330_opcode_buf *buf)
{
	/* hw req encoding: FORCE=0, SINGLE=1, BURST=3 */
	static const uint8_t enc[] = {0U, 1U, 3U};

	if (xfer > DMA_PL330_XFER_BURST) {
		return false;
	}
	if ((buf->off + 1U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x04U | enc[xfer];
	return true;
}

/**
 * @brief Emit DMAST / DMASTS / DMASTB — DMA store to destination.
 * Encoding: 1 byte.
 *
 * @param xfer  FORCE: always store; SINGLE: single req; BURST: burst req.
 * @param buf   Opcode buffer.
 * @return	    true if encoded; false if xfer is invalid or buffer is full.
 */
static inline bool dma_pl330_gen_store(enum dma_pl330_xfer xfer,
				       struct dma_pl330_opcode_buf *buf)
{
	static const uint8_t enc[] = {0U, 1U, 3U};

	if (xfer > DMA_PL330_XFER_BURST) {
		return false;
	}
	if ((buf->off + 1U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x08U | enc[xfer];
	return true;
}

/**
 * @brief Emit DMALDP — DMA load from peripheral.
 * Encoding: 2 bytes.
 *
 * @param xfer    SINGLE or BURST only.
 * @param periph  Peripheral request number (0–31).
 * @param buf     Opcode buffer.
 * @return	      true if encoded; false if xfer is invalid or buffer is full.
 */
static inline bool dma_pl330_gen_loadperiph(enum dma_pl330_xfer xfer,
					    uint8_t periph,
					    struct dma_pl330_opcode_buf *buf)
{
	uint8_t bs;

	if (xfer != DMA_PL330_XFER_SINGLE && xfer != DMA_PL330_XFER_BURST) {
		return false;
	}
	if ((buf->off + 2U) > buf->buf_size) {
		return false;
	}
	bs = (xfer == DMA_PL330_XFER_BURST) ? 1U : 0U;
	buf->buf[buf->off++] = 0x25U | (uint8_t)(bs << 1);
	buf->buf[buf->off++] = (uint8_t)((periph & 0x1FU) << 3);
	return true;
}

/**
 * @brief Emit DMASTP — DMA store to peripheral.
 * Encoding: 2 bytes.
 *
 * @param xfer    SINGLE or BURST only.
 * @param periph  Peripheral request number (0–31).
 * @param buf     Opcode buffer.
 * @return        true if encoded; false if xfer is invalid or buffer is full.
 */
static inline bool dma_pl330_gen_storeperiph(enum dma_pl330_xfer xfer,
					     uint8_t periph,
					     struct dma_pl330_opcode_buf *buf)
{
	uint8_t bs;

	if (xfer != DMA_PL330_XFER_SINGLE && xfer != DMA_PL330_XFER_BURST) {
		return false;
	}
	if ((buf->off + 2U) > buf->buf_size) {
		return false;
	}
	bs = (xfer == DMA_PL330_XFER_BURST) ? 1U : 0U;
	buf->buf[buf->off++] = 0x29U | (uint8_t)(bs << 1);
	buf->buf[buf->off++] = (uint8_t)((periph & 0x1FU) << 3);
	return true;
}

/**
 * @brief Emit DMALP — load loop counter.
 * Encoding: 2 bytes.
 *
 * @param lc    Loop counter register (LC0 or LC1).
 * @param iter  Number of iterations (1–256).
 * @param buf   Opcode buffer.
 * @return	    true if encoded; false if iter is invalid or buffer is full.
 */
static inline bool dma_pl330_gen_loop(enum dma_pl330_lc lc, uint16_t iter,
				      struct dma_pl330_opcode_buf *buf)
{
	if (iter < 1U || iter > 256U) {
		return false;
	}
	if ((buf->off + 2U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x20U | (uint8_t)((uint8_t)lc << 1);
	buf->buf[buf->off++] = (uint8_t)(iter - 1U);
	return true;
}

/**
 * @brief Emit DMALPEND / DMALPENDS / DMALPENDB — end of loop.
 * Encoding: 2 bytes.
 *
 * @param lp   Loop end arguments (xfer type, LC, jump offset, nf flag).
 * @param buf  Opcode buffer.
 * @return     true if encoded; false if buffer is full.
 *
 * @note Set lp->nf = true for standard count-controlled loops (preceded
 *       by dma_pl330_gen_loop). Set lp->jump to (current_off - loop_body_start).
 */
static inline bool dma_pl330_gen_loopend(const struct dma_pl330_loop *lp,
					 struct dma_pl330_opcode_buf *buf)
{
	/* hw burst-select encoding: FORCE=0, SINGLE=1, BURST=3 */
	static const uint8_t bs_enc[] = {0U, 1U, 3U};
	uint8_t xfer;
	uint8_t opcode;

	if ((buf->off + 2U) > buf->buf_size) {
		return false;
	}
	xfer = (lp->xfer_type >= DMA_PL330_XFER_PERIPH)
		? (uint8_t)DMA_PL330_XFER_FORCE
		: (uint8_t)lp->xfer_type;
	opcode = 0x28U
		| (lp->nf ? 0x10U : 0U)
		| (uint8_t)((uint8_t)lp->lc << 2)
		| bs_enc[xfer];
	buf->buf[buf->off++] = opcode;
	buf->buf[buf->off++] = lp->jump;
	return true;
}

/**
 * @brief Emit DMAFLUSHP — flush peripheral.
 * Encoding: 2 bytes.
 *
 * @param periph  Peripheral request number (0–31).
 * @param buf     Opcode buffer.
 * @return        true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_flushperiph(uint8_t periph,
					     struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 2U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x35U;
	buf->buf[buf->off++] = (uint8_t)((periph & 0x1FU) << 3);
	return true;
}

/**
 * @brief Emit DMAWFP — wait for peripheral.
 * Encoding: 2 bytes.
 *
 * @param xfer    SINGLE, BURST, or PERIPH (peripheral bit).
 * @param periph  Peripheral request number (0–31).
 * @param buf     Opcode buffer.
 * @return        true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_wfp(enum dma_pl330_xfer xfer, uint8_t periph,
				     struct dma_pl330_opcode_buf *buf)
{
	/* WFP opcode bits: SINGLE=0x30, PERIPH=0x31, BURST=0x32 */
	static const uint8_t wfp_enc[] = {0U, 0U, 2U, 1U};

	if ((buf->off + 2U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x30U | wfp_enc[xfer];
	buf->buf[buf->off++] = (uint8_t)((periph & 0x1FU) << 3);
	return true;
}

/**
 * @brief Emit DMAWFE — wait for event.
 * Encoding: 2 bytes.
 *
 * @param invalidate  true to invalidate the event on wait.
 * @param event_num   Event number (0–31).
 * @param buf         Opcode buffer.
 * @return            true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_wfe(bool invalidate, uint8_t event_num,
				     struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 2U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x36U;
	buf->buf[buf->off++] = (uint8_t)(((event_num & 0x1FU) << 3)
					 | (invalidate ? 0x02U : 0U));
	return true;
}

/**
 * @brief Emit DMASEV — send event / trigger interrupt.
 * Encoding: 2 bytes.
 *
 * @note For interrupt-driven transfers, event_num must equal the DMA
 *       channel number (the driver maps event_irq[ch] == ch).
 *
 * @param event_num  Event number (0–31).
 * @param buf        Opcode buffer.
 * @return           true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_send_event(uint8_t event_num,
					    struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 2U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x34U;
	buf->buf[buf->off++] = (uint8_t)((event_num & 0x1FU) << 3);
	return true;
}

/**
 * @brief Emit DMAWMB — write memory barrier.
 * Encoding: 1 byte. Use before DMASEV to ensure store visibility.
 *
 * @param buf Opcode buffer.
 * @return    true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_wmb(struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 1U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x13U;
	return true;
}

/**
 * @brief Emit DMARMB — read memory barrier.
 * Encoding: 1 byte. Ensures prior loads are visible before next load.
 *
 * @param buf Opcode buffer.
 * @return    true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_rmb(struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 1U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x12U;
	return true;
}

/**
 * @brief Emit DMANOP — no operation (for alignment).
 * Encoding: 1 byte.
 *
 * @param buf Opcode buffer.
 * @return    true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_nop(struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 1U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x18U;
	return true;
}

/**
 * @brief Emit DMAADDH — add 16-bit immediate to SAR or DAR.
 * Encoding: 3 bytes.
 *
 * @param reg      DMA_PL330_REG_SAR or DMA_PL330_REG_DAR only.
 * @param addr_off 16-bit unsigned offset to add.
 * @param buf      Opcode buffer.
 * @return	       true if encoded; false if reg is invalid or buffer is full.
 */
static inline bool dma_pl330_gen_add(enum dma_pl330_reg reg, uint16_t addr_off,
				     struct dma_pl330_opcode_buf *buf)
{
	uint8_t ar;

	if (reg != DMA_PL330_REG_SAR && reg != DMA_PL330_REG_DAR) {
		return false;
	}
	if ((buf->off + 3U) > buf->buf_size) {
		return false;
	}
	ar = (reg == DMA_PL330_REG_DAR) ? 1U : 0U;
	buf->buf[buf->off++] = 0x54U | (uint8_t)(ar << 1);
	buf->buf[buf->off++] = (uint8_t)(addr_off);
	buf->buf[buf->off++] = (uint8_t)(addr_off >> 8);
	return true;
}

/**
 * @brief Emit DMAADNH — subtract 16-bit immediate from SAR or DAR.
 * Encoding: 3 bytes. Stores the two's-complement negation.
 *
 * @param reg      DMA_PL330_REG_SAR or DMA_PL330_REG_DAR only.
 * @param addr_off 16-bit value to subtract (positive).
 * @param buf      Opcode buffer.
 * @return         true if encoded; false if reg is invalid or buffer is full.
 */
static inline bool dma_pl330_gen_addneg(enum dma_pl330_reg reg, uint16_t addr_off,
					struct dma_pl330_opcode_buf *buf)
{
	uint16_t neg;
	uint8_t ar;

	if (reg != DMA_PL330_REG_SAR && reg != DMA_PL330_REG_DAR) {
		return false;
	}
	if ((buf->off + 3U) > buf->buf_size) {
		return false;
	}
	ar  = (reg == DMA_PL330_REG_DAR) ? 1U : 0U;
	neg = (uint16_t)(~(addr_off - 1U));
	buf->buf[buf->off++] = 0x5CU | (uint8_t)(ar << 1);
	buf->buf[buf->off++] = (uint8_t)(neg);
	buf->buf[buf->off++] = (uint8_t)(neg >> 8);
	return true;
}

/**
 * @brief Emit DMASTZ — store zeros to destination (8-byte aligned burst).
 * Encoding: 1 byte.
 *
 * @param buf Opcode buffer.
 * @return    true if encoded; false if buffer is full.
 */
static inline bool dma_pl330_gen_store_zeros(struct dma_pl330_opcode_buf *buf)
{
	if ((buf->off + 1U) > buf->buf_size) {
		return false;
	}
	buf->buf[buf->off++] = 0x0CU;
	return true;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_PL330_OPCODE_H_ */
