/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ADC);

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc_common.h>
#include "analog_ctrl.h"

#define DT_DRV_COMPAT alif_adc

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

struct adc_config {
	DEVICE_MMIO_NAMED_ROM(comp_reg);
	DEVICE_MMIO_NAMED_ROM(analog_reg);
	DEVICE_MMIO_NAMED_ROM(adc_reg);
	DEVICE_MMIO_NAMED_ROM(aon_regs);
#if CONFIG_ANALOG_ALIASING
	DEVICE_MMIO_NAMED_ROM(adc_vref);
#endif
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pcfg;
	uint32_t sample_width;
	uint32_t clock_div;
	uint32_t shift_n_bits;
	uint32_t avg_sample_num;
	uint32_t drv_inst;
	uint32_t pga_gain;
	uint32_t comparator_threshold_a;
	uint32_t comparator_threshold_b;
	uint8_t shift_direction;
	uint8_t comparator_bias;
	uint8_t pga_enable;
	uint8_t scan_mode;
	uint8_t conv_mode;
	uint8_t comparator_threshold_comparison;
	uint8_t adc24_output_rate;
	uint8_t adc24_bias;
};

struct adc_data {
	DEVICE_MMIO_NAMED_RAM(comp_reg);
	DEVICE_MMIO_NAMED_RAM(analog_reg);
	DEVICE_MMIO_NAMED_RAM(adc_reg);
	DEVICE_MMIO_NAMED_RAM(aon_regs);
#if CONFIG_ANALOG_ALIASING
	DEVICE_MMIO_NAMED_RAM(adc_vref);
#endif
	const struct device *dev;
	struct adc_context ctx;
	volatile uint32_t curr_cnt;
	uint32_t *buffer;
	uint32_t buffer_size;
	uint32_t *repeat_buffer;
	uint32_t samples_count;
	uint32_t channels;
	uint32_t channels_count;
	uint8_t  interrupts;
	uint8_t *comparator;
};

enum ADC_INSTANCE {
	ADC_INSTANCE_ADC24_0,
	ADC_INSTANCE_ADC12_0,
	ADC_INSTANCE_ADC12_1,
	ADC_INSTANCE_ADC12_2,
};

#define DEV_DATA(dev) ((struct adc_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct adc_config *)((dev)->config))

/* ADC register offsets */
#define ADC_START_SRC      (0x00)
#define ADC_COMP_THRESH_A  (0x04)
#define ADC_COMP_THRESH_B  (0x08)
#define ADC_CLK_DIVISOR    (0x0C)
#define ADC_INTERRUPT      (0x10)
#define ADC_INTERRUPT_MASK (0x14)
#define ADC_SAMPLE_WIDTH   (0x18)
#define ADC_AVG_NUM        (0x20)
#define ADC_SHIFT_CONTROL  (0x24)
#define ADC_CONTROL        (0x30)
#define ADC_SEQUENCER_CTRL (0x34)
#define ADC_REG1           (0x38)
#define ADC_SEL            (0x3C)
#define ADC_PERIPH         (0x40)
#define ADC_SAMPLE_REG_0   (0x50)

/****ADC Register macros****/
#define ADC_START_CONTINUOUS_CONV          (1U << 6)
#define ADC_START_ENABLE                   (1U << 7)
#define ADC_START_SINGLE_SHOT_CONV         (1U << 0)

/****ADC Differential macros****/
#define ADC_DIFFERENTIAL_ENABLE      (0X01)
#define ADC_MAX_DIFFERENTIAL_CHANNEL (0X03)

/****ADC Differential mask macros****/
#define ADC120_DIFFERENTIAL_ENABLE (1UL << 1)
#define ADC121_DIFFERENTIAL_ENABLE (1UL << 7)
#define ADC122_DIFFERENTIAL_ENABLE (1UL << 13)
#define ADC_DIFFERENTIAL_ENABLE_MSK                                                                \
	(ADC120_DIFFERENTIAL_ENABLE | ADC121_DIFFERENTIAL_ENABLE | ADC122_DIFFERENTIAL_ENABLE)

/* Sample width */
#define ADC12_SAMPLE_WIDTH_Msk (0XFFFF)

/* ADC external trigger mask macro */
#define ADC_EXTERNAL_TRIGGER_MAX_VAL (0x3F)

/********Interrupt mask macro*******/
#define ADC_INTR_CMPA_POS            (2)
#define ADC_INTR_CMPA_MSK            (1 << ADC_INTR_CMPA_POS)
#define ADC_INTR_CMPB_POS            (3)
#define ADC_INTR_CMPB_MSK            (1 << ADC_INTR_CMPB_POS)
#define ADC_THRSHLD_CMP_MASK_BIT_POS (16)
#define ADC_THRSHLD_CMP_MASK_BIT     (0x03 << ADC_THRSHLD_CMP_MASK_BIT_POS)

/****Interrupt clear macros****/
#define ADC_INTR_DONE0_CLEAR (0x01)
#define ADC_INTR_DONE1_CLEAR (0x02)
#define ADC_INTR_COMPA_CLEAR (0x04)
#define ADC_INTR_COMPB_CLEAR (0x08)

/****Comparator Macros****/
#define ADC_CMP_THRHLD_ABOVE_A     (0)
#define ADC_CMP_THRHLD_BELOW_A     (1)
#define ADC_CMP_THRHLD_BETWEEN_A_B (2)

#define ADC_CMP_THRHLD_ABOVE_B     (0)
#define ADC_CMP_THRHLD_BELOW_B     (1)
#define ADC_CMP_THRHLD_OUTSIDE_A_B (2)

/****channels Macros****/
#define ADC_LAST_AVAILABLE_CHANNEL (8)

/****Shift bit macro****/
#define ADC_SHIFT_BIT          (16)
#define ADC_SEQUENCER_INIT_Pos (12)

/****Sequencer Macros****/
#define ADC_SEQUENCER_MSK_BIT (0x01)
#define ADC_MAX_INIT_CHANNEL  (0X100)
#define ADC_MSK_INIT_CHANNEL  (0X0F)
#define ADC_MSK_ALL_CHANNELS  (0X1FF)

/* Adc Vref setting */
#define ADC_VREF_BUF_RDIV_EN (0x0U << 16)
#define ADC_VREF_BUF_EN      (0x1U << 15)
#define ADC_VREF_CONT        (0x10U << 10)
#define ANA_PERIPH_LDO_CONT  (0xAU << 6)
#define ANA_PERIPH_BG_CONT   (0xAU << 1)

#ifdef CONFIG_ANALOG_ALIASING
/* ADC reg1 position macro */
#define ADC120_DIFFERENTIAL_EN_Pos (1)
#define ADC120_COMPARATOR_EN_Pos   (2)
#define ADC120_COMPARATOR_BIAS_Pos (3)
#define ADC120_VCM_DIV_Pos         (5)

#define ADC121_DIFFERENTIAL_EN_Pos (1)
#define ADC121_COMPARATOR_EN_Pos   (2)
#define ADC121_COMPARATOR_BIAS_Pos (3)
#define ADC121_VCM_DIV_Pos         (5)

#define ADC122_DIFFERENTIAL_EN_Pos (1)
#define ADC122_COMPARATOR_EN_Pos   (2)
#define ADC122_COMPARATOR_BIAS_Pos (3)
#define ADC122_VCM_DIV_Pos         (5)
#else
/* ADC reg1 position macro */
#define ADC120_DIFFERENTIAL_EN_Pos (1)
#define ADC120_COMPARATOR_EN_Pos   (2)
#define ADC120_COMPARATOR_BIAS_Pos (3)
#define ADC120_VCM_DIV_Pos         (5)

#define ADC121_DIFFERENTIAL_EN_Pos (7)
#define ADC121_COMPARATOR_EN_Pos   (8)
#define ADC121_COMPARATOR_BIAS_Pos (9)
#define ADC121_VCM_DIV_Pos         (11)

#define ADC122_DIFFERENTIAL_EN_Pos (13)
#define ADC122_COMPARATOR_EN_Pos   (14)
#define ADC122_COMPARATOR_BIAS_Pos (15)
#define ADC122_VCM_DIV_Pos         (17)
#endif

/* PMU_PERIPH offset */
#define PMU_PERIPH_OFFSET (0X40)

#ifdef CONFIG_ANALOG_ALIASING
/* PMU_PERIPH field definitions */
#define ADC_PERIPH_ADC1_PGA_EN           (1U << 0)
#define ADC_PERIPH_ADC1_PGA_GAIN_Pos     (1)
#define ADC_PERIPH_ADC1_PGA_GAIN_Msk     (0x7)
#define ADC_PERIPH_ADC2_PGA_EN           (1U << 0)
#define ADC_PERIPH_ADC2_PGA_GAIN_Pos     (1)
#define ADC_PERIPH_ADC2_PGA_GAIN_Msk     (0x7)
#define ADC_PERIPH_ADC3_PGA_EN           (1U << 0)
#define ADC_PERIPH_ADC3_PGA_GAIN_Pos     (1)
#define ADC_PERIPH_ADC3_PGA_GAIN_Msk     (0x7)
#define ADC_PERIPH_ADC24_EN              (1U << 12)
#define ADC_PERIPH_ADC24_OUTPUT_RATE_Pos (13)
#define ADC_PERIPH_ADC24_OUTPUT_RATE_Msk (0x7 << ADC_PERIPH_ADC24_OUTPUT_RATE_Pos)
#define ADC_PERIPH_ADC24_PGA_EN          (1U << 0)
#define ADC_PERIPH_ADC24_PGA_GAIN_Pos    (1)
#define ADC_PERIPH_ADC24_PGA_GAIN_Msk    (0x7 << ADC_PERIPH_ADC24_OUTPUT_RATE_Pos)
#define ADC_PERIPH_ADC24_BIAS_Pos        (20)
#define ADC_PERIPH_ADC24_BIAS_Msk        (0x7 << ADC_PERIPH_ADC24_BIAS_Pos)
#else
/* PMU_PERIPH field definitions */
#define PMU_PERIPH_ADC1_PGA_EN           (1U << 0)
#define PMU_PERIPH_ADC1_PGA_GAIN_Pos     (1)
#define PMU_PERIPH_ADC1_PGA_GAIN_Msk     (0x7)
#define PMU_PERIPH_ADC2_PGA_EN           (1U << 4)
#define PMU_PERIPH_ADC2_PGA_GAIN_Pos     (5)
#define PMU_PERIPH_ADC2_PGA_GAIN_Msk     (0xE0)
#define PMU_PERIPH_ADC3_PGA_EN           (1U << 8)
#define PMU_PERIPH_ADC3_PGA_GAIN_Pos     (9)
#define PMU_PERIPH_ADC3_PGA_GAIN_Msk     (0xE00)
#define PMU_PERIPH_ADC24_EN              (1U << 12)
#define PMU_PERIPH_ADC24_OUTPUT_RATE_Pos (13)
#define PMU_PERIPH_ADC24_OUTPUT_RATE_Msk (0xE000)
#define PMU_PERIPH_ADC24_PGA_EN          (1U << 16)
#define PMU_PERIPH_ADC24_PGA_GAIN_Pos    (17)
#define PMU_PERIPH_ADC24_PGA_GAIN_Msk    (0xE0000)
#define PMU_PERIPH_ADC24_BIAS_Pos        (20)
#define PMU_PERIPH_ADC24_BIAS_Msk        (0x700000)
#endif

#define COMP_REG2_OFFSET (0X04)

/* channels limit */
#define ADC24_MAX_DIFFERENTIAL_CHANNEL   (0x03)
#define ADC12_MAX_DIFFERENTIAL_CHANNEL   (0x02)
#define ADC12_MAX_CHANNELS               (0X08)

/**
 * enum _ADC_SCAN_MODE.
 * Set the scan mode for ADC conversion.
 */
enum ADC_SCAN_MODE {
	ADC_SCAN_MODE_MULTI_CH,
	ADC_SCAN_MODE_SINGLE_CH
};

/**
 * enum ADC_CONV_MODE.
 * Set the conversion mode for ADC.
 */
enum ADC_CONV_MODE {
	ADC_CONV_MODE_CONTINUOUS,
	ADC_CONV_MODE_SINGLE_SHOT
};

/**
 * enum ADC_CONV_STAT.
 * Status of an ongoing ADC conversion.
 */
enum ADC_CONV_STAT {
	ADC_CONV_STAT_CMP_THLD_ABOVE_A = (1U << 0),
	ADC_CONV_STAT_CMP_THLD_ABOVE_B = (1U << 1),
	ADC_CONV_STAT_CMP_THLD_BELOW_A = (1U << 2),
	ADC_CONV_STAT_CMP_THLD_BELOW_B = (1U << 3),
	ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B = (1U << 4),
	ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B = (1U << 5),
};

static inline void adc_clk_config(void)
{
	uintptr_t vbat_reg_base = (uintptr_t) ANA_VBAT_REG2;
	uint32_t data;

	/* CGU Enable 160 Mhz clock */
	data = sys_read32(CGU_CLK_ENA);
	data |= BIT(20);
	sys_write32(data, CGU_CLK_ENA);

	/* Enable LDO and BG for the ANALOG */
	enable_analog_peripherals(vbat_reg_base);

	/* Enable ADC Clock Control */
	data = sys_read32(CLKCTRL_PER_SLV_ADC_CTRL);
	data |= (BIT(0) | BIT(4) | BIT(8) | BIT(12));
	sys_write32(data, CLKCTRL_PER_SLV_ADC_CTRL);

	/* Enable COMP Clock Control */
	data = sys_read32(CLKCTRL_PER_SLV_CMP_CTRL);
	data |= (BIT(0) | BIT(4));
	sys_write32(data, CLKCTRL_PER_SLV_CMP_CTRL);
}

static inline void adc_enable_single_shot_conv(uintptr_t adc)
{
	uint32_t data;

	data = sys_read32(adc + ADC_START_SRC);
	data |= ADC_START_ENABLE;
	sys_write32(data, adc + ADC_START_SRC);

	data = sys_read32(adc + ADC_CONTROL);
	data |= ADC_START_SINGLE_SHOT_CONV;
	sys_write32(data, adc + ADC_CONTROL);
}

static inline void adc_disable_single_shot_conv(uintptr_t adc)
{
	uint32_t data;

	data = sys_read32(adc + ADC_START_SRC);
	data &= ~(ADC_START_ENABLE);
	sys_write32(data, adc + ADC_START_SRC);

	data = sys_read32(adc + ADC_CONTROL);
	data &= ~ADC_START_SINGLE_SHOT_CONV;
	sys_write32(data, adc + ADC_CONTROL);
}

static inline void adc_enable_continuous_conv(uintptr_t adc)
{
	uint32_t data;

	data = sys_read32(adc + ADC_START_SRC);
	data |= ADC_START_CONTINUOUS_CONV;
	sys_write32(data, adc + ADC_START_SRC);
}

static inline void adc_disable_continuous_conv(uintptr_t adc)
{
	uint32_t data;

	data = sys_read32(adc + ADC_START_SRC);
	data &= ~ADC_START_CONTINUOUS_CONV;
	sys_write32(data, adc + ADC_START_SRC);
}

static inline void adc_set_clk_div(uintptr_t adc, uint32_t divisor)
{
	sys_write32(divisor, adc + ADC_CLK_DIVISOR);
}

static inline void adc_set_avg_sample(uintptr_t adc, uint32_t average)
{
	sys_write32(average, adc + ADC_AVG_NUM);
}

static inline void adc_set_sample_width(uintptr_t adc, uint32_t width)
{
	uint32_t data;

	data = sys_read32(adc + ADC_SAMPLE_WIDTH);
	data = (data & ~ADC12_SAMPLE_WIDTH_Msk) | width;
	sys_write32(data, adc + ADC_SAMPLE_WIDTH);
}

static inline void adc_set_n_shift_bit(uintptr_t adc, uint32_t shift_number,
				       uint32_t shift_left_right_control)
{
	uint32_t data;

	data = sys_read32(adc + ADC_SHIFT_CONTROL);
	data = (shift_number | shift_left_right_control << 16);
	sys_write32(data, adc + ADC_SHIFT_CONTROL);
}

static inline void adc_set_comparator_a(uintptr_t adc, uint32_t threshold)
{
	sys_write32(threshold, adc + ADC_COMP_THRESH_A);
}

static inline void adc_set_comparator_b(uintptr_t adc, uint32_t threshold)
{
	sys_write32(threshold, adc + ADC_COMP_THRESH_B);
}

static inline void adc_set_comparator_ctrl_bit(uintptr_t adc, uint32_t arg)
{
	sys_write32((arg << 16), adc + ADC_CONTROL);
}

static inline void adc_unmask_interrupt(uintptr_t adc, uint8_t arg)
{
	sys_write32(((~arg) & 0xF), adc + ADC_INTERRUPT_MASK);
}

static inline void adc_mask_interrupt(uintptr_t adc, uint8_t arg)
{
	uint32_t data;

	data = sys_read32(adc + ADC_INTERRUPT_MASK);
	data = (~arg) & 0xF;
	sys_write32(data, adc + ADC_INTERRUPT_MASK);
}

static inline void adc_sequencer_msk_ch_control(uintptr_t adc, uint32_t mask_channel)
{
	uint32_t data;

	data = sys_read32(adc + ADC_SEQUENCER_CTRL);
	data &= ~(ADC_MSK_ALL_CHANNELS);
	data |= ((~mask_channel) & ADC_MSK_ALL_CHANNELS);
	sys_write32(data, adc + ADC_SEQUENCER_CTRL);
}

static inline void adc_init_channel_select(uintptr_t adc, uint32_t channel)
{
	uint32_t data;

	data = sys_read32(adc + ADC_SEQUENCER_CTRL);
	data &= ~(ADC_MSK_INIT_CHANNEL << ADC_SEQUENCER_INIT_Pos);
	data |= (channel << ADC_SEQUENCER_INIT_Pos);
	sys_write32(data, adc + ADC_SEQUENCER_CTRL);
}

static inline void adc_set_ch_scan_mode(const struct device *dev)
{
	uint32_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);
	const struct adc_config *config = dev->config;
	uint32_t data;

	data = sys_read32(regs + ADC_SEQUENCER_CTRL);
	data |= config->scan_mode;
	sys_write32(data, regs + ADC_SEQUENCER_CTRL);
}

static inline void enable_adc24(const struct device *dev)
{
	uint32_t data = 0;
#ifdef CONFIG_ANALOG_ALIASING
	uint32_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);

	data = sys_read32(regs + ADC_PERIPH);
	data |= ADC_PERIPH_ADC24_EN;
	sys_write32(data, (regs + ADC_PERIPH));
#else
	uintptr_t aon_ctrl = DEVICE_MMIO_NAMED_GET(dev, aon_regs);

	data = sys_read32(aon_ctrl + PMU_PERIPH_OFFSET);
	data |= PMU_PERIPH_ADC24_EN;
	sys_write32(data, (aon_ctrl + PMU_PERIPH_OFFSET));
#endif
}

static inline void set_adc24_bias(const struct device *dev, uint32_t bias)
{
	uint32_t data;
#ifdef CONFIG_ANALOG_ALIASING
	uint32_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);

	data = sys_read32(regs + ADC_PERIPH);
	data |= ((bias << ADC_PERIPH_ADC24_BIAS_Pos) & ADC_PERIPH_ADC24_BIAS_Msk);
	sys_write32(data, (regs + ADC_PERIPH));
#else
	uintptr_t aon_ctrl = DEVICE_MMIO_NAMED_GET(dev, aon_regs);

	data = sys_read32(aon_ctrl + PMU_PERIPH_OFFSET);
	data |= ((bias << PMU_PERIPH_ADC24_BIAS_Pos) & PMU_PERIPH_ADC24_BIAS_Msk);
	sys_write32(data, aon_ctrl + PMU_PERIPH_OFFSET);
#endif
}

static inline void set_adc24_output_rate(const struct device *dev, uint32_t rate)
{
	uint32_t data;
#ifdef CONFIG_ANALOG_ALIASING
	uint32_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);

	data = sys_read32(regs + ADC_PERIPH);
	data |= ((rate << ADC_PERIPH_ADC24_OUTPUT_RATE_Pos) & ADC_PERIPH_ADC24_OUTPUT_RATE_Msk);
	sys_write32(data, regs + ADC_PERIPH);
#else
	uintptr_t aon_ctrl = DEVICE_MMIO_NAMED_GET(dev, aon_regs);

	data = sys_read32(aon_ctrl + PMU_PERIPH_OFFSET);
	data |= ((rate << PMU_PERIPH_ADC24_OUTPUT_RATE_Pos) & PMU_PERIPH_ADC24_OUTPUT_RATE_Msk);
	sys_write32(data, aon_ctrl + PMU_PERIPH_OFFSET);
#endif
}

void adc_analog_config(const struct device *dev)
{

#ifdef CONFIG_ANALOG_ALIASING
	uintptr_t adc_vref_base = DEVICE_MMIO_NAMED_GET(dev, adc_vref);

	enable_adc_ref_voltage_alias_mode(adc_vref_base);
#else
	uintptr_t comp_regs = DEVICE_MMIO_NAMED_GET(dev, comp_reg);

	enable_adc_ref_voltage(comp_regs + COMP_REG2_OFFSET);
#endif
}

void adc_done0_irq_handler(const struct device *dev)
{
	uint32_t channel_sample_reg;
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);
	uint32_t sample_reg = (regs + ADC_SAMPLE_REG_0);
	uint8_t channel = sys_read32(regs + ADC_SEL);
	struct adc_data *data = dev->data;
	const struct adc_config *config = dev->config;

	/* Clearing the done0 IRQ*/
	sys_write32(ADC_INTR_DONE0_CLEAR, regs + ADC_INTERRUPT);

	if (config->conv_mode == ADC_CONV_MODE_CONTINUOUS) {
		/* storing the address to be fetched for particular channels */
		channel_sample_reg = sample_reg + sizeof(uint32_t) * channel;

		/* storing the digital output to the respective buffer cells */
		*(data->buffer + data->curr_cnt++) = sys_read32(channel_sample_reg);

		/* Check if the required number of samples has been read */
		if (data->curr_cnt >= data->channels_count) {
			data->curr_cnt = 0U;
			/* disable the interrupt*/
			adc_mask_interrupt(regs, data->interrupts);
			adc_context_on_sampling_done(&data->ctx, dev);
		}
	}
}

void adc_done1_irq_handler(const struct device *dev)
{
	uint32_t channel_sample_reg;
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);
	uintptr_t sample_reg = (regs + ADC_SAMPLE_REG_0);
	uint8_t channel = sys_read32(regs + ADC_SEL);
	struct adc_data *data = dev->data;
	const struct adc_config *config = dev->config;

	/* Clearing the done1 IRQ*/
	sys_write32(ADC_INTR_DONE1_CLEAR, regs + ADC_INTERRUPT);

	if (config->conv_mode == ADC_CONV_MODE_SINGLE_SHOT) {
		/* storing the address to be fetched for particular channels */
		channel_sample_reg = sample_reg + sizeof(uint32_t) * channel;

		/* storing the digital output to the respected buffer cells */
		*(data->buffer + data->curr_cnt++) = sys_read32(channel_sample_reg);

		/* Check if the required number of samples has been read */
		if (data->curr_cnt >= data->channels_count) {
			data->curr_cnt = 0U;
			/*disable the interrupt */
			adc_mask_interrupt(regs, data->interrupts);
			adc_context_on_sampling_done(&data->ctx, dev);
		}
	}
}

void adc_cmpa_irq_handler(const struct device *dev)
{
	uint32_t value;
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);
	struct adc_data *data = dev->data;

	/* Clearing CMPA interrupt */
	sys_write32(ADC_INTR_COMPA_CLEAR, regs + ADC_INTERRUPT);

	value = sys_read32(regs + ADC_CONTROL);
	value &= ADC_THRSHLD_CMP_MASK_BIT;
	value >>= ADC_SHIFT_BIT;

	switch (value) {
	case ADC_CMP_THRHLD_ABOVE_A:
		*(data->comparator) |= ADC_CONV_STAT_CMP_THLD_ABOVE_A;
		break;
	case ADC_CMP_THRHLD_BELOW_A:
		*(data->comparator) |= ADC_CONV_STAT_CMP_THLD_BELOW_A;
		break;
	case ADC_CMP_THRHLD_BETWEEN_A_B:
		*(data->comparator) |= ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B;
		break;
	}
}

void adc_cmpb_irq_handler(const struct device *dev)
{
	uint32_t value;
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);
	struct adc_data *data = dev->data;

	/* Clearing CMPB interrupt */
	sys_write32(ADC_INTR_COMPB_CLEAR, regs + ADC_INTERRUPT);

	value = sys_read32(regs + ADC_CONTROL);
	value &= ADC_THRSHLD_CMP_MASK_BIT;
	value >>= ADC_SHIFT_BIT;

	switch (value) {
	case ADC_CMP_THRHLD_ABOVE_B:
		*(data->comparator) |= ADC_CONV_STAT_CMP_THLD_ABOVE_B;
		break;
	case ADC_CMP_THRHLD_BELOW_B:
		*(data->comparator) |= ADC_CONV_STAT_CMP_THLD_BELOW_B;
		break;
	case ADC_CMP_THRHLD_OUTSIDE_A_B:
		*(data->comparator) |= ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B;
		break;
	}
}

void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_data *data = CONTAINER_OF(ctx, struct adc_data, ctx);
	const struct adc_config *config = data->dev->config;

	uintptr_t regs = DEVICE_MMIO_NAMED_GET(data->dev, adc_reg);

	data->repeat_buffer = data->buffer;

	adc_unmask_interrupt(regs, data->interrupts);

	if (config->conv_mode == ADC_CONV_MODE_SINGLE_SHOT) {
		adc_enable_single_shot_conv(regs);
	} else {
		adc_enable_continuous_conv(regs);
	}
}

void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_data *data = CONTAINER_OF(ctx, struct adc_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int check_buffer_size(const struct adc_sequence *sequence, uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint32_t);

	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)", sequence->buffer_size,
			needed_buffer_size);
		return -ENOMEM;
	}

	return 0;
}

static void disable_adc(const struct device *dev)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);
	const struct adc_config *config = dev->config;

	if (config->conv_mode == ADC_CONV_MODE_SINGLE_SHOT) {
		adc_disable_single_shot_conv(regs);
	} else {
		adc_disable_continuous_conv(regs);
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_data *data = CONTAINER_OF(ctx, struct adc_data, ctx);

	ARG_UNUSED(status);

	disable_adc(data->dev);
}

static int adc_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);
	struct adc_data *data = dev->data;
	uint32_t channel;
	int error;

	data->buffer = (uint32_t *)sequence->buffer;
	data->buffer_size = sequence->buffer_size;
	data->channels = sequence->channels;
	data->channels_count = POPCOUNT(data->channels);
	data->comparator = sequence->options->user_data;

	adc_sequencer_msk_ch_control(regs, sequence->channels);

	channel = ((sys_read32(regs + ADC_SEQUENCER_CTRL) & 0xF000) >> 12);

	/* check channel number passed is enabled or not */
	if (((sys_read32(regs + ADC_SEQUENCER_CTRL)) & (1 << channel))) {
		LOG_ERR(" Error: channel is masked\n");
		return -EINVAL;
	}

	if (data->channels_count == 0) {
		LOG_ERR("No channels selected");
		return -EINVAL;
	}

	if (sequence->calibrate != 0U) {
		LOG_ERR("calibrate not supported");
		return -ENOTSUP;
	}

	if (sequence->oversampling != 0U) {
		LOG_ERR("oversampling not supported");
		return -ENOTSUP;
	}

	if (sequence->resolution != 0U) {
		LOG_ERR("resolutionc not supported");
		return -ENOTSUP;
	}

	error = check_buffer_size(sequence, data->channels_count);
	if (error) {
		return error;
	}

	adc_context_start_read(&data->ctx, sequence);

	int result = adc_context_wait_for_completion(&data->ctx);

	return result;
}

static int adc_alif_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

static void adc_set_comparator_ctrl(const struct device *dev, uint8_t instance)
{
	uint32_t data = 0;
	uint32_t value = 0;

	const struct adc_config *config = dev->config;

	switch (instance) {
	case ADC_INSTANCE_ADC12_0: {
		value |= (1 << ADC120_COMPARATOR_EN_Pos |
			  config->comparator_bias << ADC120_COMPARATOR_BIAS_Pos);
		break;
	}
	case ADC_INSTANCE_ADC12_1: {
		value |= (1 << ADC121_COMPARATOR_EN_Pos |
			  config->comparator_bias << ADC121_COMPARATOR_BIAS_Pos);
		break;
	}
	case ADC_INSTANCE_ADC12_2: {
		value |= (1 << ADC122_COMPARATOR_EN_Pos |
			  config->comparator_bias << ADC122_COMPARATOR_BIAS_Pos);
		break;
	}
	default: {
		break;
	}
	}
#ifdef CONFIG_ANALOG_ALIASING
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);

	data = sys_read32(regs + ADC_REG1);
	data |= value;
	sys_write32(data, regs + ADC_REG1);
#else
	uintptr_t adc_ctrl = DEVICE_MMIO_NAMED_GET(dev, analog_reg);

	data = sys_read32(adc_ctrl + ADC_REG1);
	data |= value;
	sys_write32(data, adc_ctrl + ADC_REG1);
#endif
}

static inline void adc_set_differential_ctrl(const struct device *dev, uint8_t instance,
					     bool differential_en)
{
	uint32_t data = 0;
	uint32_t value = 0;

	switch (instance) {
	case ADC_INSTANCE_ADC12_0: {
		value |=
			(differential_en << ADC120_DIFFERENTIAL_EN_Pos | (1 << ADC120_VCM_DIV_Pos));
		break;
	}
	case ADC_INSTANCE_ADC12_1: {
		value |=
			(differential_en << ADC121_DIFFERENTIAL_EN_Pos | (1 << ADC121_VCM_DIV_Pos));
		break;
	}
	case ADC_INSTANCE_ADC12_2: {
		value |=
			(differential_en << ADC122_DIFFERENTIAL_EN_Pos | (1 << ADC122_VCM_DIV_Pos));
		break;
	}
	default: {
		break;
	}
	}

#ifdef CONFIG_ANALOG_ALIASING
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);

	data = sys_read32(regs + ADC_REG1);
	data |= value;
	sys_write32(data, regs + ADC_REG1);
#else
	uintptr_t adc_ctrl = DEVICE_MMIO_NAMED_GET(dev, analog_reg);

	data = sys_read32(adc_ctrl + ADC_REG1);
	data |= value;
	sys_write32(data, adc_ctrl + ADC_REG1);
#endif
}

static inline void enable_adc_pga_gain(const struct device *dev, uint8_t instance, uint32_t gain)
{

	uint32_t data = 0;
#ifdef CONFIG_ANALOG_ALIASING
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);

	switch (instance) {
	case ADC_INSTANCE_ADC24_0:
		sys_clear_bits((regs + ADC_PERIPH), ADC_PERIPH_ADC24_PGA_GAIN_Msk);
		data = sys_read32(regs + ADC_PERIPH);
		data |= (ADC_PERIPH_ADC24_PGA_EN | gain << ADC_PERIPH_ADC24_PGA_GAIN_Pos);
		break;
	case ADC_INSTANCE_ADC12_0:
		sys_clear_bits((regs + ADC_PERIPH), ADC_PERIPH_ADC1_PGA_GAIN_Msk);
		data = sys_read32(regs + ADC_PERIPH);
		data |= (ADC_PERIPH_ADC1_PGA_EN | gain << ADC_PERIPH_ADC1_PGA_GAIN_Pos);
		break;
	case ADC_INSTANCE_ADC12_1:
		sys_clear_bits((regs + ADC_PERIPH), ADC_PERIPH_ADC2_PGA_GAIN_Msk);
		data = sys_read32(regs + ADC_PERIPH);
		data |= (ADC_PERIPH_ADC2_PGA_EN | gain << ADC_PERIPH_ADC2_PGA_GAIN_Pos);
		break;
	case ADC_INSTANCE_ADC12_2:
		sys_clear_bits((regs + ADC_PERIPH), ADC_PERIPH_ADC3_PGA_GAIN_Msk);
		data = sys_read32(regs + ADC_PERIPH);
		data |= (ADC_PERIPH_ADC3_PGA_EN | gain << ADC_PERIPH_ADC3_PGA_GAIN_Pos);
		break;
	}

	sys_write32(data, regs + ADC_PERIPH);
#else
	uintptr_t aon_ctrl = DEVICE_MMIO_NAMED_GET(dev, aon_regs);

	switch (instance) {
	case ADC_INSTANCE_ADC24_0:
		sys_clear_bits((aon_ctrl + PMU_PERIPH_OFFSET), PMU_PERIPH_ADC24_PGA_GAIN_Msk);
		data = sys_read32(aon_ctrl + PMU_PERIPH_OFFSET);
		data |= (PMU_PERIPH_ADC24_PGA_EN | gain << PMU_PERIPH_ADC24_PGA_GAIN_Pos);
		break;
	case ADC_INSTANCE_ADC12_0:
		sys_clear_bits((aon_ctrl + PMU_PERIPH_OFFSET), PMU_PERIPH_ADC1_PGA_GAIN_Msk);
		data = sys_read32(aon_ctrl + PMU_PERIPH_OFFSET);
		data |= (PMU_PERIPH_ADC1_PGA_EN | gain << PMU_PERIPH_ADC1_PGA_GAIN_Pos);
		break;
	case ADC_INSTANCE_ADC12_1:
		sys_clear_bits((aon_ctrl + PMU_PERIPH_OFFSET), PMU_PERIPH_ADC2_PGA_GAIN_Msk);
		data = sys_read32(aon_ctrl + PMU_PERIPH_OFFSET);
		data |= (PMU_PERIPH_ADC2_PGA_EN | gain << PMU_PERIPH_ADC2_PGA_GAIN_Pos);
		break;
	case ADC_INSTANCE_ADC12_2:
		sys_clear_bits((aon_ctrl + PMU_PERIPH_OFFSET), PMU_PERIPH_ADC3_PGA_GAIN_Msk);
		data = sys_read32(aon_ctrl + PMU_PERIPH_OFFSET);
		data |= (PMU_PERIPH_ADC3_PGA_EN | gain << PMU_PERIPH_ADC3_PGA_GAIN_Pos);
		break;
	}

	sys_write32(data, aon_ctrl + PMU_PERIPH_OFFSET);
#endif
}

static int adc_channel_select(const struct device *dev, const struct adc_channel_cfg *channel_cf)
{
	const struct adc_config *config = dev->config;
	uintptr_t regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);

	if (config->drv_inst != ADC_INSTANCE_ADC24_0) {
		/* set differential control for ADC12 */
		adc_set_differential_ctrl(dev, config->drv_inst, channel_cf->differential);

		adc_set_comparator_ctrl(dev, config->drv_inst);
	}

	if (channel_cf->differential) {
		enable_adc_pga_gain(dev, config->drv_inst, config->pga_gain);
	}

	if (config->drv_inst == ADC_INSTANCE_ADC24_0) {
		if (channel_cf->channel_id > ADC24_MAX_DIFFERENTIAL_CHANNEL) {
			return -EINVAL;
		}
	} else {
		if (channel_cf->differential) {
			if (channel_cf->channel_id > ADC12_MAX_DIFFERENTIAL_CHANNEL) {
				return -EINVAL;
			}
		} else {
			if (channel_cf->channel_id > ADC12_MAX_CHANNELS) {
				return -EINVAL;
			}
		}
	}
	/* set the channel to operate */
	adc_init_channel_select(regs, channel_cf->channel_id);

	return 0;
}

static int adc_init(const struct device *dev)
{
	int err;
	const struct adc_config *config = dev->config;
	struct adc_data *data = dev->data;
	uintptr_t regs;

	DEVICE_MMIO_NAMED_MAP(dev, comp_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, adc_reg, K_MEM_CACHE_NONE);
#if CONFIG_ANALOG_ALIASING
	DEVICE_MMIO_NAMED_MAP(dev, adc_vref, K_MEM_CACHE_NONE);
#endif
	regs = DEVICE_MMIO_NAMED_GET(dev, adc_reg);

	data->dev = dev;

	adc_context_init(&data->ctx);

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	/* adc clock configuration */
	adc_clk_config();

	/* Vref setting */
	adc_analog_config(dev);

	if (config->drv_inst == ADC_INSTANCE_ADC24_0) {
		/* enable adc24 from control register */
		enable_adc24(dev);

		/* set output rate from control register */
		set_adc24_output_rate(dev, config->adc24_output_rate);

		/* Set adc24 bias from control register */
		set_adc24_bias(dev, config->adc24_bias);
	}

	/* set Sample width value for ADC12/24 */
	adc_set_sample_width(regs, config->sample_width);

	/* set the clock divisor */
	adc_set_clk_div(regs, config->clock_div);

	/* set avg sample value */
	adc_set_avg_sample(regs, config->avg_sample_num);

	/* set number of n shift bits */
	adc_set_n_shift_bit(regs, config->shift_n_bits, config->shift_direction);

	/* set comparator a threshold */
	adc_set_comparator_a(regs, config->comparator_threshold_a);

	/* set comparator b threshold */
	adc_set_comparator_b(regs, config->comparator_threshold_b);

	/* set comparator threshold operation */
	adc_set_comparator_ctrl_bit(regs, config->comparator_threshold_comparison);

	/* set channel scan mode */
	adc_set_ch_scan_mode(dev);

	/* disabling the interrupts */
	adc_unmask_interrupt(regs, data->interrupts);

	/* Install ISR handler. */
	config->irq_config_func(dev);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

struct adc_driver_api alif_adc_api = {
	.channel_setup = &adc_channel_select,
	.read = &adc_alif_read,
};

#define ADC24_BIAS_GAIN_0 0
#define ADC24_BIAS_GAIN_1 1
#define ADC24_BIAS_GAIN_2 3
#define ADC24_BIAS_GAIN_3 7

#define ADC24_BIAS(inst) UTIL_CAT(ADC24_BIAS_GAIN_, DT_INST_ENUM_IDX_OR(inst, adc24_bias, 0))

#define ADC_ALIF_INIT(inst)                                                                        \
	static void adc_config_func_##inst(const struct device *dev);                              \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, pinctrl_0), (PINCTRL_DT_INST_DEFINE(inst)));        \
	const struct adc_config config_##inst = {                                                  \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(comp_reg, DT_DRV_INST(inst)),                   \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(analog_reg, DT_DRV_INST(inst)),                 \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(adc_reg, DT_DRV_INST(inst)),                    \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(aon_regs, DT_DRV_INST(inst)),                   \
		IF_ENABLED(CONFIG_ANALOG_ALIASING,					           \
		(DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(adc_vref, DT_DRV_INST(inst)),))	           \
		.irq_config_func = adc_config_func_##inst,                                         \
		.sample_width = COND_CODE_0(DT_INST_ENUM_IDX(inst, driver_instance), (BIT(16)),    \
					    (DT_INST_ENUM_IDX(inst, sample_width))),               \
		.clock_div = DT_INST_PROP(inst, clock_div),                                        \
		.shift_n_bits = DT_INST_PROP(inst, shift_n_bits),                                  \
		.shift_direction = DT_INST_ENUM_IDX(inst, shift_direction),                        \
		.comparator_bias = COND_CODE_0(DT_INST_ENUM_IDX(inst, driver_instance), (0),       \
					       (DT_INST_ENUM_IDX(inst, comparator_bias))),         \
		.avg_sample_num = DT_INST_PROP(inst, avg_sample_num),                              \
		.drv_inst = DT_INST_ENUM_IDX(inst, driver_instance),                               \
		.pga_enable = DT_INST_PROP(inst, pga_enable),                                      \
		.pga_gain = DT_INST_ENUM_IDX(inst, pga_gain),                                      \
		.scan_mode = DT_INST_ENUM_IDX(inst, adc_channel_scan),                             \
		.conv_mode = DT_INST_ENUM_IDX(inst, adc_conversion_mode),                          \
		.comparator_threshold_a = DT_INST_PROP(inst, comparator_threshold_a),              \
		.comparator_threshold_b = DT_INST_PROP(inst, comparator_threshold_b),              \
		.comparator_threshold_comparison =                                                 \
			DT_INST_ENUM_IDX(inst, comparator_threshold_comparison),                   \
		.adc24_output_rate = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, adc24_output_rate),   \
						(DT_INST_ENUM_IDX(inst, adc24_output_rate)),       \
						(0)),                                              \
		.adc24_bias = ADC24_BIAS(inst),                                                    \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, pinctrl_0),                                 \
				(.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(inst)),))};         \
	struct adc_data data_##inst = {                                                            \
		ADC_CONTEXT_INIT_LOCK(data_##inst, ctx),                                           \
		ADC_CONTEXT_INIT_SYNC(data_##inst, ctx),                                           \
		ADC_CONTEXT_INIT_TIMER(data_##inst, ctx),                                          \
		.interrupts = DT_INST_PROP(inst, interrupt_en),                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, adc_init, NULL, &data_##inst, &config_##inst, POST_KERNEL,     \
			      CONFIG_ADC_INIT_PRIORITY, &alif_adc_api);                            \
                                                                                                   \
	static void adc_config_func_##inst(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, done0, irq),                                 \
			    DT_INST_IRQ_BY_NAME(inst, done0, priority),                            \
			    adc_done0_irq_handler, DEVICE_DT_INST_GET(inst), 0);                   \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, done1, irq),                                 \
			    DT_INST_IRQ_BY_NAME(inst, done1, priority),                            \
			    adc_done1_irq_handler, DEVICE_DT_INST_GET(inst), 0);                   \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, comp_a, irq),                                \
			    DT_INST_IRQ_BY_NAME(inst, comp_a, priority),                           \
			    adc_cmpa_irq_handler, DEVICE_DT_INST_GET(inst), 0);                    \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, comp_b, irq),                                \
			    DT_INST_IRQ_BY_NAME(inst, comp_b, priority),                           \
			    adc_cmpb_irq_handler, DEVICE_DT_INST_GET(inst), 0);                    \
                                                                                                   \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, done0, irq));                                 \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, done1, irq));                                 \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, comp_a, irq));                                \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, comp_b, irq));                                \
	}

DT_INST_FOREACH_STATUS_OKAY(ADC_ALIF_INIT)
