/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <soc_common.h>

/**
 * @brief Perform common SoC initialization at boot
 *        for ensemble family.
 *        This will run after soc_early_init_hook.
 *
 * @return 0
 */
static int soc_init(void)
{
	unsigned int data;

	/* Might need to move later.. Just putting this here for now..*/
	/* Enable UART clock and clock selection bits in CFGMST0 */
	sys_write32(0xFFFF, EXPSLV_UART_CTRL);

	/*
	 * Setting expansion master0 SPI control register values
	 * 0xf at 8-11 bit is setting all 4 SPI instances as master
	 * bit 0-3; ss_in_sel; 0 - from io pad; 1 - from ssi_in_val
	 * bit 8-11; ss_in_val; when ss_in_sel=1, feed ss_in_val to SSI,
	 * each bit controls one SSI instance.
	 * For setting an spi instance as slave, put 0 in the corresponding
	 * bit position of both 8-11 and 0-3 bit fields.
	 * For example if we want to set SPI1 as master and
	 * remaining instances as slave, set the 1st bit for ss_in_sel, which will
	 * make ss_in_val to feed to SSI, and set the corresponding ss_in_val bit.
	 * here for SPI1 as master set the 9th bit. So the value to feed SPI1 as
	 * master and remaining as slave is 0x0202.
	 */
	sys_write32(0x0202, EXPSLV_SSI_CTRL);

	data = sys_read32(M55HE_CFG_HE_CLK_ENA);
	data |= (1 << 16);
	sys_write32(data, M55HE_CFG_HE_CLK_ENA);

	/*LP-SPI Flex GPIO */
	sys_write32(0x1, VBAT_BASE);

	/* Enable DMA */
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma0), arm_dma_pl330, okay) /* dma0 */
	sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(4));
	sys_write32(0x1111, EVTRTR0_DMA_REQ_CTRL);
	sys_clear_bits(EXPMST_DMA_CTRL, BIT(0));
	sys_write32(0U, EXPMST_DMA_IRQ);
	sys_write32(0U, EXPMST_DMA_PERIPH);
	sys_set_bits(EXPMST_DMA_CTRL, BIT(16));
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma1), arm_dma_pl330, okay) /* dma1 */
	sys_set_bits(M55HP_CFG_HP_CLK_ENA, BIT(4));
	sys_write32(0x1111, EVTRTRLOCAL_DMA_REQ_CTRL);
	sys_clear_bits(M55HP_CFG_HP_DMA_CTRL, BIT(0));
	sys_write32(0U, M55HP_CFG_HP_DMA_IRQ);
	sys_write32(0U, M55HP_CFG_HP_DMA_PERIPH);
	sys_set_bits(M55HP_CFG_HP_DMA_CTRL, BIT(16));
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay) /* dma2 */
	sys_set_bits(M55HE_CFG_HE_CLK_ENA, BIT(4));
	sys_write32(0x1111, EVTRTRLOCAL_DMA_REQ_CTRL);
	sys_clear_bits(M55HE_CFG_HE_DMA_CTRL, BIT(0));
	sys_write32(0U, M55HE_CFG_HE_DMA_IRQ);
	sys_write32(0U, M55HE_CFG_HE_DMA_PERIPH);
	sys_set_bits(M55HE_CFG_HE_DMA_CTRL, BIT(16));
#endif
	/* Enable LPRTC Clock via VBAT registers */
	sys_write32(0x1, VBAT_RTC_CLK_EN);

	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, 1);
