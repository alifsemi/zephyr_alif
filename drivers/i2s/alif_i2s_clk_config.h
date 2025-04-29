/*
 * Copyright (c) 2024 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * This file provides APIs for Alif SoC specific Clock Configuration for I2S.
 * This file will be removed once the clock control driver implemented.
 */
#ifndef _ALIF_I2S_CLK_CONFIG_H_
#define _ALIF_I2S_CLK_CONFIG_H_

#define __IOM           volatile
#define __OM            volatile
#define __IM            volatile const

/**
 * enum I2S_INSTANCE
 * I2S instances
 */
typedef enum _I2S_INSTANCE {
	I2S_INSTANCE_0, /**< I2S instance - 0   */
	I2S_INSTANCE_1, /**< I2S instance - 1   */
	I2S_INSTANCE_2, /**< I2S instance - 2   */
	I2S_INSTANCE_3, /**< I2S instance - 3   */
	I2S_INSTANCE_LP /**< I2S instance - LP  */
} I2S_INSTANCE;


struct CLKCTL_PER_SLV_Type { /*!< (@ 0x4902F000) CLKCTL_PER_SLV Structure */
	/*!< (@ 0x00000000) Clock Control Register */
	__IOM uint32_t EXPMST0_CTRL;

	__IM uint32_t RESERVED;

	__IOM uint32_t UART_CTRL; /*!< (@ 0x00000008) UART Control Register */
	/*!< (@ 0x0000000C) CANFD Control Register */
	__IOM uint32_t CANFD_CTRL;
	__IOM uint32_t I2S0_CTRL; /*!< (@ 0x00000010) I2Sn Control Register */
	__IOM uint32_t I2S1_CTRL; /*!< (@ 0x00000014) I2Sn Control Register */
	__IOM uint32_t I2S2_CTRL; /*!< (@ 0x00000018) I2Sn Control Register */
	__IOM uint32_t I2S3_CTRL; /*!< (@ 0x0000001C) I2Sn Control Register */
	__IM uint32_t RESERVED1;
	__IOM uint32_t I3C_CTRL; /*!< (@ 0x00000024) I3C Control Register */
	__IOM uint32_t SSI_CTRL; /*!< (@ 0x00000028) SPI Control Register */
	__IM uint32_t RESERVED2;
	__IOM uint32_t ADC_CTRL; /*!< (@ 0x00000030) ADC Control Register */
	__IOM uint32_t DAC_CTRL; /*!< (@ 0x00000034) DAC Control Register */
	__IOM uint32_t CMP_CTRL; /*!< (@ 0x00000038) CMP Control Register */
	__IM uint32_t RESERVED3;
	/*!< (@ 0x00000040) Frequency Monitor 0 Control Register */
	__IOM uint32_t FREQ_MON_CTRL0;
	/*!< (@ 0x00000044) Frequency Monitor 0 Status Register */
	__IM uint32_t FREQ_MON_STAT0;
	/*!< (@ 0x00000048) Frequency Monitor 1 Control Register */
	__IOM uint32_t FREQ_MON_CTRL1;
	/*!< (@ 0x0000004C) Frequency Monitor 1 Status Register */
	__IM uint32_t FREQ_MON_STAT1;
	__IM uint32_t RESERVED4[12];
	/*!< (@ 0x00000080) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL0;
	/*!< (@ 0x00000084) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL1;
	/*!< (@ 0x00000088) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL2;
	/*!< (@ 0x0000008C) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL3;
	/*!< (@ 0x00000090) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL4;
	/*!< (@ 0x00000094) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL5;
	/*!< (@ 0x00000098) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL6;
	/*!< (@ 0x0000009C) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL7;
	/*!< (@ 0x000000A0) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL8;
	/*!< (@ 0x000000A4) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL9;
	/*!< (@ 0x000000A8) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL10;
	/*!< (@ 0x000000AC) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL11;
	/*!< (@ 0x000000B0) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL12;
	/*!< (@ 0x000000B4) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL13;
	/*!< (@ 0x000000B8) GPIOn Control Register */
	__IOM uint32_t GPIO_CTRL14;
}; /*!< Size = 188 (0xbc) */

struct M55HE_CFG_Type { /*!< (@ 0x43007000) M55HE_CFG Structure */
	/*!< (@ 0x00000000) DMA2 Boot Control Register */
	__IOM uint32_t HE_DMA_CTRL;
	/*!< (@ 0x00000004) DMA2 Boot IRQ Non-Secure Register */
	__IOM uint32_t HE_DMA_IRQ;
	/*!< (@ 0x00000008) DMA2 Boot Peripheral Non-Secure Register */
	__IOM uint32_t HE_DMA_PERIPH;
	/*!< (@ 0x0000000C) DMA2 Select Register */
	__IOM uint32_t HE_DMA_SEL;
	/*!< (@ 0x00000010) Peripheral Clock Enable Register */
	__IOM uint32_t HE_CLK_ENA;
	/*!< (@ 0x00000014) LPI2S Control Register */
	__IOM uint32_t HE_I2S_CTRL;
	__IM uint32_t RESERVED[2];
	/*!< (@ 0x00000020) LPCPI Pixel Clock Control Register */
	__IOM uint32_t HE_CAM_PIXCLK;
};

#define CLKCTL_PER_SLV_BASE 0x4902F000UL
#define CLKCTL_PER_SLV      ((struct CLKCTL_PER_SLV_Type *)CLKCTL_PER_SLV_BASE)

#define M55HE_CFG_BASE 0x43007000UL
#define M55HE_CFG      ((struct M55HE_CFG_Type *)M55HE_CFG_BASE)

/* CLKCTL_PER_SLV I2Sn_CTRL I2S Control field definitions */
/* SCLK Always On */
#define I2S_CTRL_SCLK_AON         (1U << 20)
/* Bypass Clock Divider */
#define I2S_CTRL_DIV_BYPASS       (1U << 17)
/* Clock Source Selection Mask */
#define I2S_CTRL_CLK_SEL_Msk      (1U << 16)
/* Enable 76.8MHz Crystal Oscillator Clock */
#define I2S_CTRL_CLK_SEL_76M8_CLK (0U << 16)
/* Enable External Audio clock input */
#define I2S_CTRL_CLK_SEL_EXT_CLK  (1U << 16)
/* Enable I2S controller clock */
#define I2S_CTRL_CKEN             (1U << 12)
/* Clock divider start position */
#define I2S_CTRL_CKDIV_Pos        (0)
/* Clock divider start mask */
#define I2S_CTRL_CKDIV_Msk        (0x3FF << I2S_CTRL_CKDIV_Pos)

static inline void enable_i2s_sclk_aon(I2S_INSTANCE instance)
{
	switch (instance) {
	case I2S_INSTANCE_0:
		CLKCTL_PER_SLV->I2S0_CTRL |= I2S_CTRL_SCLK_AON;
		break;
	case I2S_INSTANCE_1:
		CLKCTL_PER_SLV->I2S1_CTRL |= I2S_CTRL_SCLK_AON;
		break;
	case I2S_INSTANCE_2:
		CLKCTL_PER_SLV->I2S2_CTRL |= I2S_CTRL_SCLK_AON;
		break;
	case I2S_INSTANCE_3:
		CLKCTL_PER_SLV->I2S3_CTRL |= I2S_CTRL_SCLK_AON;
		break;
	case I2S_INSTANCE_LP:
		M55HE_CFG->HE_I2S_CTRL |= I2S_CTRL_SCLK_AON;
		break;
	}
}

static inline void disable_i2s_sclk_aon(I2S_INSTANCE instance)
{
	switch (instance) {
	case I2S_INSTANCE_0:
		CLKCTL_PER_SLV->I2S0_CTRL &= ~I2S_CTRL_SCLK_AON;
		break;
	case I2S_INSTANCE_1:
		CLKCTL_PER_SLV->I2S1_CTRL &= ~I2S_CTRL_SCLK_AON;
		break;
	case I2S_INSTANCE_2:
		CLKCTL_PER_SLV->I2S2_CTRL &= ~I2S_CTRL_SCLK_AON;
		break;
	case I2S_INSTANCE_3:
		CLKCTL_PER_SLV->I2S3_CTRL &= ~I2S_CTRL_SCLK_AON;
		break;
	case I2S_INSTANCE_LP:
		M55HE_CFG->HE_I2S_CTRL &= ~I2S_CTRL_SCLK_AON;
		break;
	}
}

static inline void bypass_i2s_clock_divider(I2S_INSTANCE instance)
{
	switch (instance) {
	case I2S_INSTANCE_0:
		CLKCTL_PER_SLV->I2S0_CTRL |= I2S_CTRL_DIV_BYPASS;
		break;
	case I2S_INSTANCE_1:
		CLKCTL_PER_SLV->I2S1_CTRL |= I2S_CTRL_DIV_BYPASS;
		break;
	case I2S_INSTANCE_2:
		CLKCTL_PER_SLV->I2S2_CTRL |= I2S_CTRL_DIV_BYPASS;
		break;
	case I2S_INSTANCE_3:
		CLKCTL_PER_SLV->I2S3_CTRL |= I2S_CTRL_DIV_BYPASS;
		break;
	case I2S_INSTANCE_LP:
		M55HE_CFG->HE_I2S_CTRL |= I2S_CTRL_DIV_BYPASS;
		break;
	}
}

static inline void select_i2s_clock_source(I2S_INSTANCE instance, bool ext_clk_src_enable)
{
	uint32_t val = ext_clk_src_enable ? I2S_CTRL_CLK_SEL_EXT_CLK : I2S_CTRL_CLK_SEL_76M8_CLK;

	switch (instance) {
	case I2S_INSTANCE_0:
		CLKCTL_PER_SLV->I2S0_CTRL &= ~I2S_CTRL_CLK_SEL_Msk;
		CLKCTL_PER_SLV->I2S0_CTRL |= val;
		break;
	case I2S_INSTANCE_1:
		CLKCTL_PER_SLV->I2S1_CTRL &= ~I2S_CTRL_CLK_SEL_Msk;
		CLKCTL_PER_SLV->I2S1_CTRL |= val;
		break;
	case I2S_INSTANCE_2:
		CLKCTL_PER_SLV->I2S2_CTRL &= ~I2S_CTRL_CLK_SEL_Msk;
		CLKCTL_PER_SLV->I2S2_CTRL |= val;
		break;
	case I2S_INSTANCE_3:
		CLKCTL_PER_SLV->I2S3_CTRL &= ~I2S_CTRL_CLK_SEL_Msk;
		CLKCTL_PER_SLV->I2S3_CTRL |= val;
		break;
	case I2S_INSTANCE_LP:
		M55HE_CFG->HE_I2S_CTRL &= ~I2S_CTRL_CLK_SEL_Msk;
		M55HE_CFG->HE_I2S_CTRL |= val;
		break;
	}
}

static inline void enable_i2s_clock(I2S_INSTANCE instance)
{
	switch (instance) {
	case I2S_INSTANCE_0:
		CLKCTL_PER_SLV->I2S0_CTRL |= I2S_CTRL_CKEN;
		break;
	case I2S_INSTANCE_1:
		CLKCTL_PER_SLV->I2S1_CTRL |= I2S_CTRL_CKEN;
		break;
	case I2S_INSTANCE_2:
		CLKCTL_PER_SLV->I2S2_CTRL |= I2S_CTRL_CKEN;
		break;
	case I2S_INSTANCE_3:
		CLKCTL_PER_SLV->I2S3_CTRL |= I2S_CTRL_CKEN;
		break;
	case I2S_INSTANCE_LP:
		M55HE_CFG->HE_I2S_CTRL |= I2S_CTRL_CKEN;
		break;
	}
}

static inline void disable_i2s_clock(I2S_INSTANCE instance)
{
	switch (instance) {
	case I2S_INSTANCE_0:
		CLKCTL_PER_SLV->I2S0_CTRL &= ~I2S_CTRL_CKEN;
		break;
	case I2S_INSTANCE_1:
		CLKCTL_PER_SLV->I2S1_CTRL &= ~I2S_CTRL_CKEN;
		break;
	case I2S_INSTANCE_2:
		CLKCTL_PER_SLV->I2S2_CTRL &= ~I2S_CTRL_CKEN;
		break;
	case I2S_INSTANCE_3:
		CLKCTL_PER_SLV->I2S3_CTRL &= ~I2S_CTRL_CKEN;
		break;
	case I2S_INSTANCE_LP:
		M55HE_CFG->HE_I2S_CTRL &= ~I2S_CTRL_CKEN;
		break;
	}
}

static inline void set_i2s_clock_divisor(I2S_INSTANCE instance, uint16_t clk_div)
{
	switch (instance) {
	case I2S_INSTANCE_0:
		CLKCTL_PER_SLV->I2S0_CTRL &= ~I2S_CTRL_CKDIV_Msk;
		CLKCTL_PER_SLV->I2S0_CTRL |= (clk_div & I2S_CTRL_CKDIV_Msk);
		break;
	case I2S_INSTANCE_1:
		CLKCTL_PER_SLV->I2S1_CTRL &= ~I2S_CTRL_CKDIV_Msk;
		CLKCTL_PER_SLV->I2S1_CTRL |= (clk_div & I2S_CTRL_CKDIV_Msk);
		break;
	case I2S_INSTANCE_2:
		CLKCTL_PER_SLV->I2S2_CTRL &= ~I2S_CTRL_CKDIV_Msk;
		CLKCTL_PER_SLV->I2S2_CTRL |= (clk_div & I2S_CTRL_CKDIV_Msk);
		break;
	case I2S_INSTANCE_3:
		CLKCTL_PER_SLV->I2S3_CTRL &= ~I2S_CTRL_CKDIV_Msk;
		CLKCTL_PER_SLV->I2S3_CTRL |= (clk_div & I2S_CTRL_CKDIV_Msk);
		break;
	case I2S_INSTANCE_LP:
		M55HE_CFG->HE_I2S_CTRL &= ~I2S_CTRL_CKDIV_Msk;
		M55HE_CFG->HE_I2S_CTRL |= (clk_div & I2S_CTRL_CKDIV_Msk);
		break;
	}
}
#endif /* _ALIF_I2S_CLK_CONFIG_H_*/

