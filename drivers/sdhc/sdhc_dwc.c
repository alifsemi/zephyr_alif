/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dwc_sdhc

#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sys/util.h>

#include "sdhc_dwc.h"

#ifdef CONFIG_SDHC_DWC_DMA_ADDR_TRANSLATE
#if defined(CONFIG_SOC_FAMILY_ENSEMBLE) || defined(CONFIG_SOC_FAMILY_BALLETTO)
#include "soc_memory_map.h"
#define SDHC_DMA_ADDR(p) local_to_global(p)
#else
#define SDHC_DMA_ADDR(p) ((mem_addr_t)(p))
#endif
#else
#define SDHC_DMA_ADDR(p) ((mem_addr_t)(p))
#endif

LOG_MODULE_REGISTER(sdhc_dwc, CONFIG_SDHC_LOG_LEVEL);

struct sdhc_dwc_config {
	struct dwc_sdhc_regs *regs;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
	uint32_t max_bus_freq;
	uint32_t min_bus_freq;
	uint32_t power_delay_ms;
	uint8_t bus_width;
	bool no_1_8_v;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec cd_gpio;
};

struct sdhc_dwc_data {
	struct sdhc_host_props props;
	struct k_sem lock;
	struct k_event irq_event;
	struct sdhc_io ios;
	sdhc_interrupt_cb_t callback;
	void *user_data;
#if defined(CONFIG_SDHC_DWC_ADMA)
	adma2_desc_t *adma_desc;
#endif
};

#define ERR_INTR_STATUS_EVENT(bits) ((uint32_t)(bits) << 16)

static int sdhc_dwc_hw_reset(const struct device *dev, uint8_t reset_mask);

static void sdhc_dwc_clear_interrupts(const struct device *dev)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;

	regs->DWC_SDHC_ERROR_INT_STAT_R = DWC_SDHC_ERROR_INTR_ALL_Msk;
	regs->DWC_SDHC_NORMAL_INT_STAT_R = DWC_SDHC_NORM_INTR_ALL_Msk;
}

static void sdhc_dwc_dump_regs(const struct device *dev)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;

	LOG_DBG("========== SDHC Register Dump ==========");
	LOG_DBG("  PSTATE   : 0x%08x", regs->DWC_SDHC_PSTATE_REG);
	LOG_DBG("  HC1      : 0x%02x      PWR    : 0x%02x",
		regs->DWC_SDHC_HOST_CTRL1_R, regs->DWC_SDHC_PWR_CTRL_R);
	LOG_DBG("  CLK      : 0x%04x    HC2    : 0x%04x",
		regs->DWC_SDHC_CLK_CTRL_R, regs->DWC_SDHC_HOST_CTRL2_R);
	LOG_DBG("  XFER     : 0x%04x    CMD    : 0x%04x",
		regs->DWC_SDHC_XFER_MODE_R, regs->DWC_SDHC_CMD_R);
	LOG_DBG("  BLKSZ    : 0x%04x    BLKCNT : 0x%04x",
		regs->DWC_SDHC_BLOCKSIZE_R, regs->DWC_SDHC_BLOCKCOUNT_R);
	LOG_DBG("  NIS      : 0x%04x    EIS    : 0x%04x",
		regs->DWC_SDHC_NORMAL_INT_STAT_R, regs->DWC_SDHC_ERROR_INT_STAT_R);
	LOG_DBG("  NIS_EN   : 0x%04x    EIS_EN : 0x%04x",
		regs->DWC_SDHC_NORMAL_INT_STAT_EN_R, regs->DWC_SDHC_ERROR_INT_STAT_EN_R);
	LOG_DBG("  RESP[0]  : 0x%08x  RESP[1]: 0x%08x",
		regs->DWC_SDHC_RESP01_R, regs->DWC_SDHC_RESP23_R);
	LOG_DBG("  RESP[2]  : 0x%08x  RESP[3]: 0x%08x",
		regs->DWC_SDHC_RESP45_R, regs->DWC_SDHC_RESP67_R);
#if defined(CONFIG_SDHC_DWC_ADMA)
	LOG_DBG("  ADMA_SA  : 0x%08x  ADMA_ERR: 0x%02x",
		regs->DWC_SDHC_ADMA_SA_LOW_R, regs->DWC_SDHC_ADMA_ERR_STAT_R);
#elif defined(CONFIG_SDHC_DWC_SDMA)
	LOG_DBG("  SDMASA   : 0x%08x", regs->DWC_SDHC_SDMASA_R);
#endif
	LOG_DBG("  ACMD_STAT: 0x%04x",
		regs->DWC_SDHC_AUTO_CMD_STAT_R);
	LOG_DBG("========== SDHC Dump End ================");
}

static bool sdhc_dwc_disable_clock(struct dwc_sdhc_regs *regs)
{
	uint32_t pstate = regs->DWC_SDHC_PSTATE_REG;

	if (pstate & DWC_SDHC_CMD_INHIBIT_Msk) {
		LOG_ERR("Cannot disable clock: CMD inhibit");
		return false;
	}
	if (pstate & DWC_SDHC_DAT_INHIBIT_Msk) {
		LOG_ERR("Cannot disable clock: DAT inhibit");
		return false;
	}

	regs->DWC_SDHC_CLK_CTRL_R &= ~(DWC_SDHC_CLK_EN_Msk | DWC_SDHC_INTERNAL_CLK_EN_Msk);

	return true;
}

static bool sdhc_dwc_enable_clock(struct dwc_sdhc_regs *regs)
{
	int timeout = DWC_SDHC_CLK_STABLE_TIMEOUT_US;

	regs->DWC_SDHC_CLK_CTRL_R |= DWC_SDHC_INTERNAL_CLK_EN_Msk;

	while (!(regs->DWC_SDHC_CLK_CTRL_R & DWC_SDHC_INTERNAL_CLK_STABLE_Msk) &&
	       (timeout > 0)) {
		k_busy_wait(1);
		timeout--;
	}

	if (timeout <= 0) {
		LOG_ERR("Internal clock not stable");
		return false;
	}

	regs->DWC_SDHC_CLK_CTRL_R |= DWC_SDHC_CLK_EN_Msk;

	return true;
}

static int sdhc_dwc_set_voltage(struct dwc_sdhc_regs *regs, enum sd_voltage voltage)
{
	uint32_t timeout = DWC_SDHC_1P8V_TIMEOUT_US;

	/* Power off */
	regs->DWC_SDHC_PWR_CTRL_R &= ~DWC_SDHC_PC_BUS_PWR_VDD1_Msk;

	switch (voltage) {
	case SD_VOL_3_3_V:
		regs->DWC_SDHC_PWR_CTRL_R = DWC_SDHC_PC_BUS_VSEL_3V3_Msk;
		regs->DWC_SDHC_HOST_CTRL2_R &= ~DWC_SDHC_HOST_CTRL2_SIGNALING_EN_Msk;
		regs->DWC_SDHC_PWR_CTRL_R |= DWC_SDHC_PC_BUS_PWR_VDD1_Msk;
		break;

	case SD_VOL_3_0_V:
		regs->DWC_SDHC_PWR_CTRL_R = DWC_SDHC_PC_BUS_VSEL_3V0_Msk;
		regs->DWC_SDHC_HOST_CTRL2_R &= ~DWC_SDHC_HOST_CTRL2_SIGNALING_EN_Msk;
		regs->DWC_SDHC_PWR_CTRL_R |= DWC_SDHC_PC_BUS_PWR_VDD1_Msk;
		break;

	case SD_VOL_1_8_V:
		regs->DWC_SDHC_PWR_CTRL_R = DWC_SDHC_PC_BUS_VSEL_1V8_Msk;
		regs->DWC_SDHC_HOST_CTRL2_R |= DWC_SDHC_HOST_CTRL2_SIGNALING_EN_Msk;
		k_busy_wait(DWC_SDHC_1P8V_TIMEOUT_US);

		regs->DWC_SDHC_PWR_CTRL_R |= DWC_SDHC_PC_BUS_PWR_VDD1_Msk;

		/* Re-enable clock and wait 1ms before checking line levels */
		regs->DWC_SDHC_CLK_CTRL_R |= DWC_SDHC_CLK_EN_Msk;
		k_busy_wait(1000);

		while (!(regs->DWC_SDHC_PSTATE_REG & DWC_SDHC_CMD_LINE_LVL_UP_Msk) &&
				--timeout) {
			k_busy_wait(1);
		}
		if (!timeout) {
			return -ETIMEDOUT;
		}
		break;

	default:
		return -EINVAL;
	}

	LOG_DBG("Voltage set to %s (pwr_ctrl=0x%02x)",
		voltage == SD_VOL_3_3_V ? "3.3V" :
		voltage == SD_VOL_3_0_V ? "3.0V" :
		voltage == SD_VOL_1_8_V ? "1.8V" : "unknown",
		regs->DWC_SDHC_PWR_CTRL_R);

	return 0;
}

static void sdhc_dwc_set_power(struct dwc_sdhc_regs *regs, enum sdhc_power state)
{
	if (state == SDHC_POWER_ON) {
		regs->DWC_SDHC_PWR_CTRL_R |= DWC_SDHC_PC_BUS_PWR_VDD1_Msk;
		k_msleep(5);
	} else {
		regs->DWC_SDHC_PWR_CTRL_R &= ~DWC_SDHC_PC_BUS_PWR_VDD1_Msk;
	}

	LOG_DBG("Power %s (pwr_ctrl=0x%02x)",
		state == SDHC_POWER_ON ? "ON" : "OFF",
		regs->DWC_SDHC_PWR_CTRL_R);
}

static int sdhc_dwc_clock_set(struct dwc_sdhc_regs *regs, uint32_t freq_hz)
{
	uint32_t base_clk_mhz;
	uint32_t div;
	uint16_t clk_val;
	uint32_t base_clk_hz;
	uint32_t timeout = DWC_SDHC_CLK_STABLE_TIMEOUT_US;

	/* Read base clock from capabilities */
	base_clk_mhz = (regs->DWC_SDHC_CAPABILITIES1_R & DWC_SDHC_BASE_CLK_FREQ_Msk)
			>> DWC_SDHC_BASE_CLK_FREQ_Pos;
	if (base_clk_mhz == 0) {
		base_clk_mhz = DWC_SDHC_DEFAULT_BASE_CLK_MHZ;
	}

	base_clk_hz = MHZ(base_clk_mhz);

	/* SD clock divider: SD Clock Frequency = Base Clock / (2 * div)
	 * div = 0 means base clock directly
	 */
	if (freq_hz >= base_clk_hz) {
		div = 0;
	} else {
		div = (base_clk_hz + (2 * freq_hz) - 1) / (2 * freq_hz);
		if (div > 0x3FF) {
			div = 0x3FF;
		}
	}

	if (!sdhc_dwc_disable_clock(regs)) {
		return -EIO;
	}

	/* Build clock control value:
	 * Use divided clock mode + PLL enable.
	 * Write divider + PLL_EN + INTERNAL_CLK_EN in one shot,
	 * then wait for stable and enable SD clock.
	 */
	clk_val = (uint16_t)((div & 0xFF) << DWC_SDHC_FREQ_SEL_Pos);
	clk_val |= (uint16_t)(((div >> 8) & 0x03) << DWC_SDHC_UPPER_FREQ_SEL_Pos);
	clk_val |= DWC_SDHC_PLL_EN_Msk | DWC_SDHC_INTERNAL_CLK_EN_Msk;

	regs->DWC_SDHC_CLK_CTRL_R = clk_val;

	/* Wait for internal clock stable */
	while (!(regs->DWC_SDHC_CLK_CTRL_R & DWC_SDHC_INTERNAL_CLK_STABLE_Msk) &&
	       (timeout > 0)) {
		k_busy_wait(1);
		timeout--;
	}

	if (timeout <= 0) {
		LOG_ERR("Internal clock not stable");
		return -EIO;
	}

	/* Enable SD clock output only after internal clock is stable */
	regs->DWC_SDHC_CLK_CTRL_R |= DWC_SDHC_CLK_EN_Msk;

	/* clock stabilized, still ensure 1ms delay for clock stability */
	k_busy_wait(1000);

	/* Enable High Speed mode or UHS timing based on requested frequency */
	if (freq_hz >= MHZ(50)) {
		/* Enable High Speed in Host Control 1 */
		regs->DWC_SDHC_HOST_CTRL1_R |= DWC_SDHC_HOST_CTRL1_HS_EN_Msk;

		/* UHS modes only apply with 1.8V signaling */
		if (regs->DWC_SDHC_HOST_CTRL2_R & DWC_SDHC_HOST_CTRL2_SIGNALING_EN_Msk) {
			uint16_t uhs = (freq_hz > MHZ(50))
				       ? DWC_SDHC_HOST_CTRL2_UHS_MODE_SDR104
				       : DWC_SDHC_HOST_CTRL2_UHS_MODE_SDR25;

			regs->DWC_SDHC_HOST_CTRL2_R =
				(regs->DWC_SDHC_HOST_CTRL2_R &
				 ~DWC_SDHC_HOST_CTRL2_UHS_MODE_Msk) | uhs;
		}
	} else {
		/* Below 50MHz: disable High Speed, clear UHS mode */
		regs->DWC_SDHC_HOST_CTRL1_R &= ~DWC_SDHC_HOST_CTRL1_HS_EN_Msk;
		regs->DWC_SDHC_HOST_CTRL2_R &=
			~DWC_SDHC_HOST_CTRL2_UHS_MODE_Msk;
	}

	LOG_DBG("Clock set: %uHz CLK_CTRL: 0x%04x HC1: 0x%02x HC2: 0x%04x",
		div ? (base_clk_hz / (2 * div)) : base_clk_hz,
		regs->DWC_SDHC_CLK_CTRL_R,
		regs->DWC_SDHC_HOST_CTRL1_R,
		regs->DWC_SDHC_HOST_CTRL2_R);

	return 0;
}

static uint8_t sdhc_dwc_encode_resp(uint8_t rsp_type)
{
	switch (rsp_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_NONE:
		return 0x00;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
		return DWC_SDHC_RESP_R48 | DWC_SDHC_CMD_R_CRC_CHK_EN_Msk |
		       DWC_SDHC_CMD_R_CMD_IDX_CHK_EN_Msk;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		return DWC_SDHC_RESP_R48B | DWC_SDHC_CMD_R_CRC_CHK_EN_Msk |
		       DWC_SDHC_CMD_R_CMD_IDX_CHK_EN_Msk;
	case SD_RSP_TYPE_R2:
		return DWC_SDHC_RESP_R136 | DWC_SDHC_CMD_R_CRC_CHK_EN_Msk;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		return DWC_SDHC_RESP_R48;
	default:
		return 0x00;
	}
}

static int sdhc_dwc_wait_cmd_complete(const struct device *dev,
				       struct sdhc_dwc_data *data,
				       uint32_t timeout_ms)
{
	uint32_t events;
	k_timeout_t wait;
	int ret = 0;

	if (timeout_ms == SDHC_TIMEOUT_FOREVER) {
		wait = K_FOREVER;
	} else {
		wait = K_MSEC(timeout_ms ? timeout_ms : 1000);
	}

	events = k_event_wait(&data->irq_event,
			      DWC_SDHC_INTR_CC_Msk |
			      ERR_INTR_STATUS_EVENT(DWC_SDHC_ERROR_INTR_ALL_Msk),
			      false, wait);

	if (events & DWC_SDHC_INTR_CC_Msk) {
		return 0;
	}

	if (events & ERR_INTR_STATUS_EVENT(DWC_SDHC_ERROR_INTR_ALL_Msk)) {
		LOG_ERR("CMD error event: 0x%08x", events);
		ret = -EIO;
	} else {
		LOG_ERR("CMD complete timeout");
		ret = -ETIMEDOUT;
	}

	sdhc_dwc_hw_reset(dev, DWC_SDHC_SW_RST_CMD_Msk | DWC_SDHC_SW_RST_DAT_Msk);
	return ret;
}

static int sdhc_dwc_wait_xfr_complete(const struct device *dev,
				       struct sdhc_dwc_data *data,
				       uint32_t timeout_ms)
{
	uint32_t events;
	k_timeout_t wait;
	int ret = 0;

	if (timeout_ms == SDHC_TIMEOUT_FOREVER) {
		wait = K_FOREVER;
	} else {
		wait = K_MSEC(timeout_ms ? timeout_ms : 1000);
	}

	events = k_event_wait(&data->irq_event,
			      DWC_SDHC_INTR_TC_Msk |
			      ERR_INTR_STATUS_EVENT(DWC_SDHC_ERROR_INTR_ALL_Msk),
			      false, wait);

	if (events & DWC_SDHC_INTR_TC_Msk) {
		return 0;
	}

	if (events & ERR_INTR_STATUS_EVENT(DWC_SDHC_ERROR_INTR_ALL_Msk)) {
		LOG_ERR("XFR complete error event: 0x%08x", events);
		ret = -EIO;
	} else {
		LOG_ERR("XFR complete timeout");
		ret = -ETIMEDOUT;
	}

	sdhc_dwc_hw_reset(dev, DWC_SDHC_SW_RST_CMD_Msk | DWC_SDHC_SW_RST_DAT_Msk);
	return ret;
}

static void sdhc_dwc_read_response(struct dwc_sdhc_regs *regs, struct sdhc_command *cmd)
{
	if (cmd->response_type == SD_RSP_TYPE_NONE) {
		return;
	}

	if ((cmd->response_type & SDHC_NATIVE_RESPONSE_MASK) == SD_RSP_TYPE_R2) {
		cmd->response[0] = regs->DWC_SDHC_RESP01_R;
		cmd->response[1] = regs->DWC_SDHC_RESP23_R;
		cmd->response[2] = regs->DWC_SDHC_RESP45_R;
		cmd->response[3] = regs->DWC_SDHC_RESP67_R;

		if (IS_ENABLED(CONFIG_SDHC_RSP_136_HAS_CRC)) {
			for (int i = 0; i < 4; i++) {
				cmd->response[i] <<= 8;
				if (i != 3) {
					cmd->response[i] |=
						cmd->response[i + 1] >> 24;
				}
			}
		}
	} else {
		cmd->response[0] = regs->DWC_SDHC_RESP01_R;
	}
}

static int sdhc_dwc_hw_reset(const struct device *dev, uint8_t reset_mask)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	uint32_t reset_timeout = DWC_SDHC_SW_RST_TIMEOUT;

	regs->DWC_SDHC_SW_RST_R = reset_mask;
	while (regs->DWC_SDHC_SW_RST_R != 0 && --reset_timeout) {
		k_busy_wait(1);
	}

	if (!reset_timeout) {
		LOG_ERR("SDHC reset timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

static bool sdhc_dwc_is_cmd_data_idle(const struct device *dev,
				struct sdhc_command *cmd, bool data_present)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	int64_t deadline;
	uint32_t timeout_ms;

	/* Handle SDHC_TIMEOUT_FOREVER - use a reasonable max timeout */
	if (cmd->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		timeout_ms = DWC_SDHC_DATA_TIMEOUT_MS;
	} else {
		timeout_ms = cmd->timeout_ms;
	}

	deadline = k_uptime_get() + timeout_ms;

	/* Check CMD line idle */
	while (regs->DWC_SDHC_PSTATE_REG & DWC_SDHC_CMD_INHIBIT_Msk) {
		if (k_uptime_get() >= deadline) {
			return false;
		}
		k_msleep(1);
	}

	/* Check DATA line idle */
	if (data_present) {
		deadline = k_uptime_get() + DWC_SDHC_DATA_TIMEOUT_MS;

		while (regs->DWC_SDHC_PSTATE_REG & DWC_SDHC_DAT_INHIBIT_Msk) {
			if (k_uptime_get() >= deadline) {
				return false;
			}
			k_msleep(1);
		}
	}

	return true;
}

static int sdhc_dwc_send_cmd(const struct device *dev, struct sdhc_dwc_data *data,
				struct sdhc_command *cmd, bool data_present)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	uint16_t cmd_reg;
	int ret;

	/* Clear interrupt status */
	sdhc_dwc_clear_interrupts(dev);

	if (!sdhc_dwc_is_cmd_data_idle(dev, cmd, data_present)) {
		return -EBUSY;
	}

	/* Clear events before issuing command */
	k_event_clear(&data->irq_event,
		      DWC_SDHC_INTR_CC_Msk | DWC_SDHC_INTR_TC_Msk |
		      ERR_INTR_STATUS_EVENT(DWC_SDHC_ERROR_INTR_ALL_Msk));

	/* Set argument and issue command */
	regs->DWC_SDHC_ARGUMENT_R = cmd->arg;

	cmd_reg = (cmd->opcode << DWC_SDHC_CMD_IDX_Pos) | sdhc_dwc_encode_resp(cmd->response_type);
	if (data_present) {
		cmd_reg |= DWC_SDHC_CMD_R_DATA_PRES_SEL_Msk;
	}

	/* Set command register */
	regs->DWC_SDHC_CMD_R = cmd_reg;

	/* Wait for command complete */
	ret = sdhc_dwc_wait_cmd_complete(dev, data, cmd->timeout_ms);

	LOG_DBG("CMD: 0x%04x ARG: 0x%08x XFER: 0x%04x RSP01: 0x%08x "
	       "PSTATE: 0x%08x cc:%d",
	       cmd_reg, cmd->arg, regs->DWC_SDHC_XFER_MODE_R, regs->DWC_SDHC_RESP01_R,
	       regs->DWC_SDHC_PSTATE_REG, !ret);

	if (ret) {
		LOG_ERR("CMD: 0x%04x ARG: 0x%08x XFER: 0x%04x RSP01: 0x%08x "
			"PSTATE: 0x%08x, cc:%d",
			cmd_reg, cmd->arg, regs->DWC_SDHC_XFER_MODE_R, regs->DWC_SDHC_RESP01_R,
			regs->DWC_SDHC_PSTATE_REG, !ret);

		sdhc_dwc_hw_reset(dev, DWC_SDHC_SW_RST_CMD_Msk);
		return ret;
	}

	sdhc_dwc_read_response(regs, cmd);

	return 0;
}

static int sdhc_dwc_reset(const struct device *dev)
{
	return sdhc_dwc_hw_reset(dev, DWC_SDHC_SW_RST_ALL_Msk);
}

static int sdhc_dwc_dma_init(struct dwc_sdhc_regs *regs, struct sdhc_dwc_data *data,
			      struct sdhc_data *sdhc_data, bool read)
{
	uint32_t total_len = sdhc_data->blocks * sdhc_data->block_size;

	if (IS_ENABLED(CONFIG_CACHE_MANAGEMENT) && !read) {
		sys_cache_data_flush_range(sdhc_data->data, total_len);
	}

#if defined(CONFIG_SDHC_DWC_ADMA)
	uint32_t remaining = total_len;
	uint32_t offset = 0;
	uint32_t desc_num = 0;

	while (remaining > 0 && desc_num < CONFIG_SDHC_DWC_ADMA_MAX_DESC) {
		uint32_t chunk = (remaining > DWC_SDHC_ADMA2_DESC_MAX_LEN)
				 ? DWC_SDHC_ADMA2_DESC_MAX_LEN : remaining;
		mem_addr_t addr = SDHC_DMA_ADDR((const volatile void *)
						((uintptr_t)sdhc_data->data + offset));

		/* Validate that address fits in 32-bit register (ADMA2 uses 32-bit addresses) */
		if (addr > UINT32_MAX) {
			LOG_ERR("DMA address 0x%lx exceeds 32-bit limit", addr);
			return -ERANGE;
		}

		data->adma_desc[desc_num] = DWC_SDHC_ADMA2_DESC(
			DWC_SDHC_ADMA2_DESC_TRAN | DWC_SDHC_ADMA2_DESC_VALID,
			(uint16_t)chunk, addr);
		remaining -= chunk;
		offset += chunk;
		desc_num++;
	}

	if (desc_num == 0) {
		LOG_ERR("No ADMA2 descriptors built");
		return -EINVAL;
	}

	if (remaining > 0) {
		LOG_ERR("ADMA2 descriptor table overflow: %u descriptors insufficient",
			CONFIG_SDHC_DWC_ADMA_MAX_DESC);
		return -ENOMEM;
	}

	data->adma_desc[desc_num - 1] |= DWC_SDHC_ADMA2_DESC_END;

	if (IS_ENABLED(CONFIG_CACHE_MANAGEMENT)) {
		sys_cache_data_flush_range(data->adma_desc, desc_num * sizeof(adma2_desc_t));
	}

	regs->DWC_SDHC_ADMA_SA_LOW_R =
		SDHC_DMA_ADDR((const volatile void *)&data->adma_desc[0]);

#elif defined(CONFIG_SDHC_DWC_SDMA)
	/* SDMA: program buffer address into SDMA System Address register */
	mem_addr_t sdma_addr = SDHC_DMA_ADDR((const volatile void *)sdhc_data->data);

	/* Validate SDMA address fits in 32-bit */
	if (sdma_addr > UINT32_MAX) {
		LOG_ERR("SDMA address 0x%lx exceeds 32-bit limit", sdma_addr);
		return -ERANGE;
	}
	regs->DWC_SDHC_SDMASA_R = sdma_addr;

	LOG_DBG("SDMA: addr=0x%08x len=%u", regs->DWC_SDHC_SDMASA_R, total_len);
#endif

	return 0;
}

static int sdhc_dwc_init_xfr(const struct device *dev, struct sdhc_command *cmd,
			      struct sdhc_data *sdhc_data, bool read)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	uint16_t xfer_mode = 0;
	int ret;

	ret = sdhc_dwc_dma_init(regs, dev->data, sdhc_data, read);
	if (ret) {
		return ret;
	}

	/* DMA engine selected in set_def_config */
	xfer_mode |= DWC_SDHC_XFER_MODE_DMA_EN_Msk;

	regs->DWC_SDHC_BLOCKSIZE_R = (uint16_t)(sdhc_data->block_size & 0xFFF);
	regs->DWC_SDHC_BLOCKCOUNT_R = (uint16_t)sdhc_data->blocks;

	if (sdhc_data->blocks > 1) {
		xfer_mode |= DWC_SDHC_XFER_MODE_MULTI_BLK_SEL_Msk |
			     DWC_SDHC_XFER_MODE_BLK_CNT_Msk;

		if ((cmd->opcode == SD_READ_MULTIPLE_BLOCK ||
		     cmd->opcode == SD_WRITE_MULTIPLE_BLOCK)) {
			xfer_mode |= DWC_SDHC_XFER_MODE_AUTO_CMD12 <<
				     DWC_SDHC_XFER_MODE_AUTO_CMD_EN_Pos;
		}
	} else {
		xfer_mode |= DWC_SDHC_XFER_MODE_BLK_CNT_Msk;
	}

	if (read) {
		xfer_mode |= DWC_SDHC_XFER_MODE_DATA_XFER_RD_Msk;
	}

	regs->DWC_SDHC_XFER_MODE_R = xfer_mode;
	regs->DWC_SDHC_TOUT_CTRL_R = sdhc_dwc_ms_to_tout(sdhc_data->timeout_ms);

	return 0;
}

static int sdhc_dwc_send_cmd_no_data(const struct device *dev,
				      struct sdhc_command *cmd)
{
	return sdhc_dwc_send_cmd(dev, dev->data, cmd, false);
}

static int sdhc_dwc_send_cmd_data(const struct device *dev,
				   struct sdhc_command *cmd,
				   struct sdhc_data *sdhc_data, bool read)
{
	struct sdhc_dwc_data *data = dev->data;
	int ret;

	ret = sdhc_dwc_init_xfr(dev, cmd, sdhc_data, read);
	if (ret) {
		LOG_ERR("Init XFR failed: %d", ret);
		return ret;
	}

	ret = sdhc_dwc_send_cmd(dev, data, cmd, true);
	if (ret) {
		sdhc_dwc_hw_reset(dev, DWC_SDHC_SW_RST_CMD_Msk);
		return ret;
	}

	/* Wait for transfer complete */
	ret = sdhc_dwc_wait_xfr_complete(dev, data, sdhc_data->timeout_ms);
	if (ret) {
		LOG_ERR("XFR wait failed for CMD%u: %d", cmd->opcode, ret);
		sdhc_dwc_dump_regs(dev);
		sdhc_dwc_hw_reset(dev, DWC_SDHC_SW_RST_DAT_Msk);
		return ret;
	}

	/* Invalidate cache */
	if (read && IS_ENABLED(CONFIG_CACHE_MANAGEMENT)) {
		uint32_t total_len = sdhc_data->blocks * sdhc_data->block_size;

		sys_cache_data_invd_range(sdhc_data->data, total_len);
	}

	return 0;
}

static int sdhc_dwc_request(const struct device *dev, struct sdhc_command *cmd,
			     struct sdhc_data *data_req)
{
	struct sdhc_dwc_data *data = dev->data;
	int ret;

	k_sem_take(&data->lock, K_FOREVER);

	if (data_req) {
		bool read;

		switch (cmd->opcode) {
		case SD_WRITE_SINGLE_BLOCK:
		case SD_WRITE_MULTIPLE_BLOCK:
			read = false;
			break;
		case SDIO_RW_EXTENDED:
			read = !(cmd->arg & BIT(31));
			break;
		default:
			read = true;
			break;
		}

		ret = sdhc_dwc_send_cmd_data(dev, cmd, data_req, read);
	} else {
		ret = sdhc_dwc_send_cmd_no_data(dev, cmd);
	}

	k_sem_give(&data->lock);
	return ret;
}

static int sdhc_dwc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct sdhc_dwc_data *data = dev->data;
	struct dwc_sdhc_regs *regs = config->regs;

	if (ios->bus_width != data->ios.bus_width) {
		uint8_t hc1 = regs->DWC_SDHC_HOST_CTRL1_R;

		hc1 &= ~(DWC_SDHC_HOST_CTRL1_DATA_WIDTH_4BIT_Msk |
			  DWC_SDHC_HOST_CTRL1_EXT_DATA_WIDTH_Msk);

		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			break;
		case SDHC_BUS_WIDTH4BIT:
			hc1 |= DWC_SDHC_HOST_CTRL1_DATA_WIDTH_4BIT_Msk;
			break;
		case SDHC_BUS_WIDTH8BIT:
			hc1 |= DWC_SDHC_HOST_CTRL1_EXT_DATA_WIDTH_Msk;
			break;
		default:
			return -ENOTSUP;
		}

		regs->DWC_SDHC_HOST_CTRL1_R = hc1;
	}

	if (ios->power_mode != data->ios.power_mode) {
		if (ios->power_mode == SDHC_POWER_OFF) {
			sdhc_dwc_set_power(regs, SDHC_POWER_OFF);
		} else {
			sdhc_dwc_set_power(regs, SDHC_POWER_ON);
			sdhc_dwc_enable_clock(regs);
		}
	}

	if (ios->signal_voltage != data->ios.signal_voltage) {
		int ret;

		regs->DWC_SDHC_CLK_CTRL_R &= ~DWC_SDHC_CLK_EN_Msk;

		ret = sdhc_dwc_set_voltage(regs, ios->signal_voltage);
		if (ret) {
			return ret;
		}

		sdhc_dwc_enable_clock(regs);
	}

	if (ios->clock != data->ios.clock) {
		if (ios->clock == 0) {
			sdhc_dwc_disable_clock(regs);
			data->ios = *ios;
			return 0;
		}

		int ret = sdhc_dwc_clock_set(regs, ios->clock);

		if (ret) {
			LOG_ERR("Failed to set clock to %u Hz", ios->clock);
			return ret;
		}
	}

	data->ios = *ios;
	return 0;
}

static int sdhc_dwc_get_card_present(const struct device *dev)
{
	const struct sdhc_dwc_config *config = dev->config;
	int ret;

	/* Use cd-gpios for card detect if available, else fall back to register */
	if (config->cd_gpio.port) {
		ret = gpio_pin_get_dt(&config->cd_gpio);
		if (ret < 0) {
			LOG_ERR("Failed to read CD GPIO");
			return ret;
		}
		return (ret == 1) ? 1 : 0;
	}

	return (config->regs->DWC_SDHC_PSTATE_REG & DWC_SDHC_CARD_INSRT_Msk) ? 1 : 0;
}

static int sdhc_dwc_card_busy(const struct device *dev)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	uint32_t pstate, dat_line_status;

	uint32_t reg = regs->DWC_SDHC_PSTATE_REG;

	pstate = reg & (DWC_SDHC_DAT_INHIBIT_Msk | DWC_SDHC_CMD_INHIBIT_Msk);
	dat_line_status = reg & DWC_SDHC_CMD_DATA_LINE_STATUS_Msk;

	if (pstate) {
		return true;        /* CMD or DAT inhibit set -> busy */
	} else if (!dat_line_status) {
		return true;        /* DAT lines low -> card is busy */
	} else {
		return false;        /* DAT lines high -> not busy */
	}
}

static int sdhc_dwc_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct sdhc_dwc_data *data = dev->data;
	struct dwc_sdhc_regs *regs = config->regs;
	uint32_t cap1 = regs->DWC_SDHC_CAPABILITIES1_R;

	memset(props, 0, sizeof(struct sdhc_host_props));

	props->f_max = config->max_bus_freq;
	props->f_min = config->min_bus_freq;
	props->power_delay = config->power_delay_ms;

	props->host_caps.vol_330_support = !!(cap1 & DWC_SDHC_CAP1_VOL_3V3_Msk);
	props->host_caps.vol_300_support = !!(cap1 & DWC_SDHC_CAP1_VOL_3V0_Msk);
	props->host_caps.vol_180_support = config->no_1_8_v ? 0 :
						  !!(cap1 & DWC_SDHC_CAP1_VOL_1V8_Msk);
	props->host_caps.bus_4_bit_support = (config->bus_width >= 4);
	props->host_caps.bus_8_bit_support = (config->bus_width >= 8) &&
						    !!(cap1 & DWC_SDHC_CAP1_8BIT_SUPPORT_Msk);
	props->host_caps.high_spd_support = !!(cap1 & DWC_SDHC_CAP1_HIGH_SPEED_Msk);
	props->host_caps.adma_2_support = !!(cap1 & DWC_SDHC_CAP1_ADMA2_SUPPORT_Msk);
	props->host_caps.sdma_support = !!(cap1 & DWC_SDHC_CAP1_SDMA_SUPPORT_Msk);
	props->host_caps.suspend_res_support = !!(cap1 & DWC_SDHC_CAP1_SUSPEND_RESUME_Msk);
	props->host_caps.max_blk_len = (cap1 & DWC_SDHC_CAP1_MAX_BLK_LEN_Msk)
					>> DWC_SDHC_CAP1_MAX_BLK_LEN_Pos;
	props->host_caps.sdio_async_interrupt_support = !!(cap1 & DWC_SDHC_CAP1_ASYNC_IRQ_Msk);

	data->props = *props;

	return 0;
}

static int sdhc_dwc_enable_interrupt(const struct device *dev,
				    sdhc_interrupt_cb_t callback,
				    int sources, void *user_data)
{
	struct sdhc_dwc_data *data = dev->data;
	uint16_t stat_en = 0;
	uint16_t sig_en = 0;

	data->callback = callback;
	data->user_data = user_data;

	if (sources & SDHC_INT_SDIO) {
		stat_en |= DWC_SDHC_INTR_CARD_Msk;
		sig_en |= DWC_SDHC_INTR_CARD_Msk;
	}
	if (sources & SDHC_INT_INSERTED) {
		stat_en |= DWC_SDHC_INTR_CARD_INSRT_Msk;
		sig_en |= DWC_SDHC_INTR_CARD_INSRT_Msk;
	}
	if (sources & SDHC_INT_REMOVED) {
		stat_en |= DWC_SDHC_INTR_CARD_REM_Msk;
		sig_en |= DWC_SDHC_INTR_CARD_REM_Msk;
	}

	/* re-enable the card interrupts */
	const struct sdhc_dwc_config *config = dev->config;

	config->regs->DWC_SDHC_NORMAL_INT_STAT_EN_R |= stat_en;
	config->regs->DWC_SDHC_NORMAL_INT_SIGNAL_EN_R |= sig_en;

	return 0;
}

static int sdhc_dwc_disable_interrupt(const struct device *dev, int sources)
{
	uint16_t mask = 0;

	if (sources & SDHC_INT_SDIO) {
		mask |= DWC_SDHC_INTR_CARD_Msk;
	}
	if (sources & SDHC_INT_INSERTED) {
		mask |= DWC_SDHC_INTR_CARD_INSRT_Msk;
	}
	if (sources & SDHC_INT_REMOVED) {
		mask |= DWC_SDHC_INTR_CARD_REM_Msk;
	}

	const struct sdhc_dwc_config *config = dev->config;

	config->regs->DWC_SDHC_NORMAL_INT_STAT_EN_R &= ~mask;
	config->regs->DWC_SDHC_NORMAL_INT_SIGNAL_EN_R &= ~mask;

	return 0;
}

static int sdhc_dwc_execute_tuning(const struct device *dev)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	uint32_t timeout = DWC_SDHC_TUNING_TIMEOUT_MS;

	LOG_DBG("Tuning starting...");

	/* Set Execute Tuning bit to start the tuning procedure */
	regs->DWC_SDHC_HOST_CTRL2_R |= DWC_SDHC_HOST_CTRL2_EXEC_TUNING_Msk;

	/* Wait for controller to clear Execute Tuning (tuning complete) */
	while ((regs->DWC_SDHC_HOST_CTRL2_R & DWC_SDHC_HOST_CTRL2_EXEC_TUNING_Msk) &&
	       --timeout) {
		k_msleep(1);
	}

	if (!timeout) {
		LOG_ERR("Tuning timed out");
		return -ETIMEDOUT;
	}

	/* Check if Sampling Clock Select is set (tuning successful) */
	if (regs->DWC_SDHC_HOST_CTRL2_R & DWC_SDHC_HOST_CTRL2_SAMPLING_CLK_Msk) {
		LOG_DBG("Tuning completed successfully");
		return 0;
	}

	LOG_ERR("Tuning failed");
	return -EIO;
}

static DEVICE_API(sdhc, sdhc_dwc_api) = {
	.reset = sdhc_dwc_reset,
	.request = sdhc_dwc_request,
	.set_io = sdhc_dwc_set_io,
	.get_card_present = sdhc_dwc_get_card_present,
	.card_busy = sdhc_dwc_card_busy,
	.get_host_props = sdhc_dwc_get_host_props,
	.enable_interrupt = sdhc_dwc_enable_interrupt,
	.disable_interrupt = sdhc_dwc_disable_interrupt,
	.execute_tuning = sdhc_dwc_execute_tuning,
};

static int sdhc_dwc_set_def_config(const struct device *dev)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	int ret;

	/* Set default configuration */

	/* Set bus voltage to 3.3V and enable power */
	ret = sdhc_dwc_set_voltage(regs, SD_VOL_3_3_V);
	if (ret != 0) {
		return ret;
	}
	sdhc_dwc_set_power(regs, SDHC_POWER_ON);

	/* Set data timeout */
	regs->DWC_SDHC_TOUT_CTRL_R = DWC_SDHC_MAX_TIMEOUT;

	/* Configure interrupt status enable (all normal except card IRQ) */
	regs->DWC_SDHC_NORMAL_INT_STAT_EN_R = DWC_SDHC_NORM_INTR_ALL_Msk & ~DWC_SDHC_INTR_CARD_Msk;
	regs->DWC_SDHC_ERROR_INT_STAT_EN_R = DWC_SDHC_ERROR_INTR_ALL_Msk;

	/* Enable interrupt signals for CC, TC, DMA, BWR, BRR */
	regs->DWC_SDHC_NORMAL_INT_SIGNAL_EN_R = DWC_SDHC_INTR_CC_Msk | DWC_SDHC_INTR_TC_Msk |
				DWC_SDHC_INTR_DMA_Msk | DWC_SDHC_INTR_BWR_Msk |
				DWC_SDHC_INTR_BRR_Msk;

	/* Enable card interrupt signal if wakeup IRQ is defined */
	if (DT_INST_IRQ_HAS_NAME(n, sdmmc_wakeup)) {
		regs->DWC_SDHC_NORMAL_INT_SIGNAL_EN_R |= DWC_SDHC_INTR_CARD_Msk |
			DWC_SDHC_INTR_CARD_REM_Msk;
	}
	regs->DWC_SDHC_ERROR_INT_SIGNAL_EN_R = DWC_SDHC_ERROR_INTR_ALL_Msk;

	/* Configure Host Controller Version 4 */
	regs->DWC_SDHC_HOST_CTRL2_R = DWC_SDHC_HOST_CTRL2_ASYNC_INT_EN_Msk |
				    DWC_SDHC_HOST_CTRL2_VER4_EN_Msk;

	/* Configure DMA mode */
#if defined(CONFIG_SDHC_DWC_ADMA)
	regs->DWC_SDHC_HOST_CTRL1_R =
		(regs->DWC_SDHC_HOST_CTRL1_R & ~DWC_SDHC_HOST_CTRL1_DMA_SEL_Msk) |
		DWC_SDHC_HOST_CTRL1_DMA_SEL_ADMA32;
#elif defined(CONFIG_SDHC_DWC_SDMA)
	regs->DWC_SDHC_HOST_CTRL1_R =
		(regs->DWC_SDHC_HOST_CTRL1_R & ~DWC_SDHC_HOST_CTRL1_DMA_SEL_Msk) |
		DWC_SDHC_HOST_CTRL1_DMA_SEL_SDMA;
#endif

	/* Enable wakeup on Card Insert and remove interrupt*/
	regs->DWC_SDHC_WUP_CTRL_R = (DWC_SDHC_WKUP_CARD_INSRT_Msk | DWC_SDHC_WKUP_CARD_REM_Msk);

	/* Set initial clock to 400KHz */
	return sdhc_dwc_clock_set(regs, DWC_SDHC_CLK_400_KHZ);
}

static int sdhc_dwc_init(const struct device *dev)
{
	const struct sdhc_dwc_config *config = dev->config;
	struct sdhc_dwc_data *data = dev->data;

	k_sem_init(&data->lock, 1, 1);
	k_event_init(&data->irq_event);

	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		return ret;
	}

	if (config->cd_gpio.port) {
		if (!gpio_is_ready_dt(&config->cd_gpio)) {
			LOG_ERR("cd-gpios not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->cd_gpio, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
	} else {
		LOG_DBG("cd-gpios not available, using PSTATE register for card detect");
	}

	if (config->reset_gpio.port) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			LOG_ERR("reset-gpio not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_LOW);
		if (ret < 0) {
			return ret;
		}
		k_msleep(100);

		/* Power on / de-assert reset */
		gpio_pin_set_dt(&config->reset_gpio, 1);
		k_msleep(100);
		gpio_pin_set_dt(&config->reset_gpio, 0);
		k_msleep(100);
	}

	/* Software reset all */
	ret = sdhc_dwc_reset(dev);
	if (ret) {
		return ret;
	}

	ret = sdhc_dwc_set_def_config(dev);
	if (ret) {
		return ret;
	}

	config->irq_config_func(dev);

	/* Populate host props from capabilities registers */
	sdhc_dwc_get_host_props(dev, &data->props);

	return 0;
}

static void sdhc_dwc_isr(const struct device *dev)
{
	struct sdhc_dwc_data *data = dev->data;
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	uint16_t eis;
	uint16_t nis;

	eis = regs->DWC_SDHC_ERROR_INT_STAT_R;
	nis = regs->DWC_SDHC_NORMAL_INT_STAT_R;

	/* Error Interrupts */
	if (eis) {
		regs->DWC_SDHC_ERROR_INT_STAT_R = eis;
		regs->DWC_SDHC_NORMAL_INT_STAT_R = nis;
		LOG_DBG("ERR_INT_STAT: 0x%04x NIS: 0x%04x", eis, nis);
		sdhc_dwc_dump_regs(dev);
		k_event_post(&data->irq_event, ERR_INTR_STATUS_EVENT(eis));
		return;
	}

	/* Command Complete */
	if (nis & DWC_SDHC_INTR_CC_Msk) {
		regs->DWC_SDHC_NORMAL_INT_STAT_R = DWC_SDHC_INTR_CC_Msk;
		k_event_post(&data->irq_event, DWC_SDHC_INTR_CC_Msk);
	}

	/* Transfer Complete */
	if (nis & DWC_SDHC_INTR_TC_Msk) {
		regs->DWC_SDHC_NORMAL_INT_STAT_R = DWC_SDHC_INTR_TC_Msk;
		k_event_post(&data->irq_event, DWC_SDHC_INTR_TC_Msk);
	}

	/* DMA Interrupt (SDMA boundary or ADMA descriptor interrupt) */
	if (nis & DWC_SDHC_INTR_DMA_Msk) {
		regs->DWC_SDHC_NORMAL_INT_STAT_R = DWC_SDHC_INTR_DMA_Msk;
#if defined(CONFIG_SDHC_DWC_SDMA)
		/*
		 * SDMA boundary: re-program SDMA System Address to continue.
		 * The HW pauses at each system address boundary. Writing
		 * SDMASA with the current value resumes the transfer.
		 */
		regs->DWC_SDHC_SDMASA_R = regs->DWC_SDHC_SDMASA_R;
#endif
		k_event_post(&data->irq_event, DWC_SDHC_INTR_DMA_Msk);
	}

	/* SDIO Card Interrupt (level-triggered) */
	if (nis & DWC_SDHC_INTR_CARD_Msk) {
		regs->DWC_SDHC_NORMAL_INT_STAT_R = DWC_SDHC_INTR_CARD_Msk;
		regs->DWC_SDHC_NORMAL_INT_STAT_EN_R &= ~DWC_SDHC_INTR_CARD_Msk;
		regs->DWC_SDHC_NORMAL_INT_SIGNAL_EN_R &= ~DWC_SDHC_INTR_CARD_Msk;

		if (data->callback) {
			data->callback(dev, SDHC_INT_SDIO,
				       data->user_data ? data->user_data : NULL);
		}
	}

	/* Clear any remaining normal interrupt status bits */
	if (nis & ~(DWC_SDHC_INTR_CC_Msk | DWC_SDHC_INTR_TC_Msk |
		    DWC_SDHC_INTR_DMA_Msk | DWC_SDHC_INTR_CARD_Msk)) {
		regs->DWC_SDHC_NORMAL_INT_STAT_R = nis;
	}
}

static void sdhc_dwc_wakeup_isr(const struct device *dev)
{
	struct sdhc_dwc_data *data = dev->data;
	const struct sdhc_dwc_config *config = dev->config;
	struct dwc_sdhc_regs *regs = config->regs;
	uint16_t nis = regs->DWC_SDHC_NORMAL_INT_STAT_R;

	if (nis & DWC_SDHC_INTR_CARD_INSRT_Msk) {
		regs->DWC_SDHC_NORMAL_INT_STAT_R = DWC_SDHC_INTR_CARD_INSRT_Msk;
		if (data->callback) {
			data->callback(dev, SDHC_INT_INSERTED,
				       data->user_data ? data->user_data : NULL);
		}
	}

	if (nis & DWC_SDHC_INTR_CARD_REM_Msk) {
		regs->DWC_SDHC_NORMAL_INT_STAT_R = DWC_SDHC_INTR_CARD_REM_Msk;
		if (data->callback) {
			data->callback(dev, SDHC_INT_REMOVED,
				       data->user_data ? data->user_data : NULL);
		}
	}
}

#define SDHC_DWC_INIT(n)                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                     \
                                                                                   \
	static void sdhc_dwc_irq_config_func_##n(const struct device *dev)            \
	{                                                                              \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, sdmmc, irq),                            \
			    DT_INST_IRQ_BY_NAME(n, sdmmc, priority),                           \
			    sdhc_dwc_isr, DEVICE_DT_INST_GET(n), 0);                          \
		irq_enable(DT_INST_IRQ_BY_NAME(n, sdmmc, irq));                            \
                                                                                   \
		if (DT_INST_IRQ_HAS_NAME(n, sdmmc_wakeup)) {                               \
			IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, sdmmc_wakeup, irq),                 \
				    DT_INST_IRQ_BY_NAME(n, sdmmc_wakeup, priority),              \
				    sdhc_dwc_wakeup_isr, DEVICE_DT_INST_GET(n), 0);              \
			irq_enable(DT_INST_IRQ_BY_NAME(n, sdmmc_wakeup, irq));                 \
		}                                                                          \
	}                                                                              \
                                                                                   \
	IF_ENABLED(CONFIG_SDHC_DWC_ADMA, (                                            \
		static adma2_desc_t sdhc_adma_desc_tbl_##n[CONFIG_SDHC_DWC_ADMA_MAX_DESC]  \
			Z_GENERIC_SECTION(CONFIG_SDHC_DESCRIPTOR_SECTION)                      \
			__aligned(CONFIG_SDHC_BUFFER_ALIGNMENT);                               \
	))                                                                             \
	static struct sdhc_dwc_data sdhc_dwc_data_##n = {                             \
		IF_ENABLED(CONFIG_SDHC_DWC_ADMA, (                                         \
			.adma_desc = sdhc_adma_desc_tbl_##n,                                   \
		))                                                                         \
	};                               \
                                                                                   \
	static const struct sdhc_dwc_config sdhc_dwc_config_##n = {                   \
		.regs = (struct dwc_sdhc_regs *)DT_INST_REG_ADDR(n),               \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                 \
		.irq_config_func = sdhc_dwc_irq_config_func_##n,                           \
		.max_bus_freq = DT_INST_PROP_OR(n, max_bus_freq, 50000000),                \
		.min_bus_freq = DT_INST_PROP_OR(n, min_bus_freq, 400000),                  \
		.power_delay_ms = DT_INST_PROP_OR(n, power_delay_ms, 500),                 \
		.bus_width = DT_INST_PROP_OR(n, bus_width, 4),                             \
		.no_1_8_v = DT_INST_PROP_OR(n, no_1_8_v, false),                           \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),               \
		.cd_gpio = GPIO_DT_SPEC_INST_GET_OR(n, cd_gpios, {0}),                     \
	};                                                                             \
                                                                                   \
	DEVICE_DT_INST_DEFINE(n, sdhc_dwc_init, NULL, &sdhc_dwc_data_##n,             \
			      &sdhc_dwc_config_##n, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,   \
			      &sdhc_dwc_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_DWC_INIT)
