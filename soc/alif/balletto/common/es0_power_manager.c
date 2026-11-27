/*
 * Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include "es0_power_manager.h"
#include "se_service.h"
#include "alif_protocol_const.h"

static volatile uint8_t es0_user_counter;

#define LL_BOOT_PARAMS_MAX_SIZE (512)

#define LL_CLK_SEL_CTRL_REG_ADDR   0x1A60201C
#define LL_UART_CLK_SEL_CTRL_16MHZ 0x00
#define LL_UART_CLK_SEL_CTRL_24MHZ 0x01
#define LL_UART_CLK_SEL_CTRL_48MHZ 0x03

/* Tag status: (STATUS_VALID | STATUS_NOT_LOCKED | STATUS_NOT_ERASED) */
#define DEFAULT_TAG_STATUS (0x00 | 0x02 | 0x04)

/* Boot time value definitions */
#define BOOT_PARAM_ID_LE_CODED_PHY_500          0x85
#define BOOT_PARAM_ID_DFT_SLAVE_MD              0x20
#define BOOT_PARAM_ID_CH_CLASS_REP_INTV         0x36
#define BOOT_PARAM_ID_BD_ADDRESS                0x01
#define BOOT_PARAM_ID_ACTIVITY_MOVE_CONFIG      0x15
#define BOOT_PARAM_ID_SCAN_EXT_ADV              0x16
#define BOOT_PARAM_ID_RSSI_HIGH_THR             0x3A
#define BOOT_PARAM_ID_RSSI_LOW_THR              0x3B
#define BOOT_PARAM_ID_SLEEP_ENABLE              0x11
#define BOOT_PARAM_ID_EXT_WAKEUP_ENABLE         0x12
#define BOOT_PARAM_ID_ENABLE_CHANNEL_ASSESSMENT 0x19
#define BOOT_PARAM_ID_RSSI_INTERF_THR           0x3C
#define BOOT_PARAM_ID_UART_BAUDRATE             0x10
#define BOOT_PARAM_ID_UART_INPUT_CLK_FREQ       0xC0
#define BOOT_PARAM_ID_NO_PARAM                  0xFF
#define BOOT_PARAM_ID_EXT_WAKEUP_TIME           0x0D
#define BOOT_PARAM_ID_OSC_WAKEUP_TIME           0x0E
#define BOOT_PARAM_ID_RM_WAKEUP_TIME            0x0F
#define BOOT_PARAM_ID_EXT_WARMBOOT_WAKEUP_TIME  0xD0
#define BOOT_PARAM_ID_LPCLK_DRIFT               0x07
#define BOOT_PARAM_ID_ACTCLK_DRIFT              0x09
#define BOOT_PARAM_ID_CONFIGURATION             0xD1
#define BOOT_PARAM_ID_CONFIGURATION2            0xD2

#define BOOT_PARAM_LEN_LE_CODED_PHY_500          1
#define BOOT_PARAM_LEN_DFT_SLAVE_MD              1
#define BOOT_PARAM_LEN_CH_CLASS_REP_INTV         2
#define BOOT_PARAM_LEN_BD_ADDRESS                6
#define BOOT_PARAM_LEN_ACTIVITY_MOVE_CONFIG      1
#define BOOT_PARAM_LEN_SCAN_EXT_ADV              1
#define BOOT_PARAM_LEN_RSSI_THR                  1
#define BOOT_PARAM_LEN_SLEEP_ENABLE              1
#define BOOT_PARAM_LEN_EXT_WAKEUP_ENABLE         1
#define BOOT_PARAM_LEN_ENABLE_CHANNEL_ASSESSMENT 1
#define BOOT_PARAM_LEN_UART_BAUDRATE             4
#define BOOT_PARAM_LEN_UART_INPUT_CLK_FREQ       4
#define BOOT_PARAM_LEN_EXT_WAKEUP_TIME           2
#define BOOT_PARAM_LEN_OSC_WAKEUP_TIME           2
#define BOOT_PARAM_LEN_RM_WAKEUP_TIME            2
#define BOOT_PARAM_LEN_EXT_WARMBOOT_WAKEUP_TIME  2
#define BOOT_PARAM_LEN_LPCLK_DRIFT               2
#define BOOT_PARAM_LEN_ACTCLK_DRIFT              1
#define BOOT_PARAM_LEN_CONFIGURATION             4
#define BOOT_PARAM_LEN_CONFIGURATION2            4

#define CONFIGURATION_RF_TYPE_HPA		 1
#define CONFIGURATION_SOC_TYPE_CSP		 2
#define CONFIGURATION_LIMIT_TX_POWER		 4
/* Bit 4 is RFU */
/* Bits 5-12 are used for TX power */
/* Bits 13-6 are RFU */

#define ES0_PM_ERROR_NO_ERROR             0
#define ES0_PM_ERROR_TOO_MANY_USERS       -1
#define ES0_PM_ERROR_TOO_MANY_BOOT_PARAMS -2
#define ES0_PM_ERROR_INVALID_BOOT_PARAMS  -3
#define ES0_PM_ERROR_START_FAILED         -4
#define ES0_PM_ERROR_NO_BAUDRATE          -5
#define ES0_PM_ERROR_BAUDRATE_MISMATCH    -6

struct es0_start_params_t {
	bool hpa_enabled;
};

static struct es0_start_params_t es0_start_params = {
	.hpa_enabled = IS_ENABLED(CONFIG_ALIF_HPA_MODE),
};

static uint8_t *write_tlv_int(uint8_t *target, uint8_t tag, uint32_t value, uint8_t len)
{
	*target++ = tag;
	*target++ = DEFAULT_TAG_STATUS;
	*target++ = len;
	if (len == 1) {
		memcpy(target, (uint8_t *)&value, len);
	} else if (len == 2) {
		memcpy(target, (uint16_t *)&value, len);
	} else {
		memcpy(target, &value, len);
	}

	target += len;

	return target;
}

static uint8_t *write_tlv_str(uint8_t *target, uint8_t tag, const void *value, uint8_t len)
{
	*target++ = tag;
	*target++ = DEFAULT_TAG_STATUS;
	*target++ = len;

	memcpy(target, value, len);

	target += len;

	return target;
}

static const void *bdaddr_reverse(const uint8_t src[6])
{
	static uint8_t rev[6];

	for (int i = 0; i < 6; ++i) {
		rev[i] = src[5 - i];
	}

	return rev;
}

static void alif_eui48_read(uint8_t *eui48)
{
#ifdef ALIF_IEEE_MA_L_IDENTIFIER
	eui48[0] = (uint8_t)(ALIF_IEEE_MA_L_IDENTIFIER >> 16);
	eui48[1] = (uint8_t)(ALIF_IEEE_MA_L_IDENTIFIER >> 8);
	eui48[2] = (uint8_t)(ALIF_IEEE_MA_L_IDENTIFIER);
#else
	se_service_get_rnd_num(&eui48[0], 3);
	eui48[0] |= 0xC0;
#endif
	se_system_get_eui_extension(true, &eui48[3]);
	if (eui48[3] || eui48[4] || eui48[5]) {
		return;
	}
	/* Generate Random Local value (ELI) */
	se_service_get_rnd_num(&eui48[3], 3);
}

static uint16_t add_nvds_param_length(uint16_t added_len)
{
	return (added_len + 3); /* Each write_tlv_x call writes additional 3 bytes */

}

void es0_enable_hpa_mode(bool enabled)
{
	es0_start_params.hpa_enabled = enabled;
}

int8_t take_es0_into_use_with_params(uint8_t *nvds_buff, uint16_t nvds_size, uint32_t clock_select,
				     bool hpa_mode)
{
	if (255 == es0_user_counter) {
		return ES0_PM_ERROR_TOO_MANY_USERS;
	} else if (es0_user_counter == 0) {
		/* Start */
		if (se_service_boot_es0(nvds_buff, nvds_size, clock_select, hpa_mode)) {
			return ES0_PM_ERROR_START_FAILED;
		}
	}

	es0_user_counter++;
	return ES0_PM_ERROR_NO_ERROR;
}

int8_t take_es0_into_use(void)
{

	uint32_t hci_baudrate;
	uint32_t ahi_baudrate;
	uint32_t used_baudrate;

	hci_baudrate = DT_PROP_OR(DT_CHOSEN(zephyr_hci_uart), current_speed, 0);
	ahi_baudrate = DT_PROP_OR(DT_CHOSEN(zephyr_ahi_uart), current_speed, 0);

	if (!hci_baudrate && !ahi_baudrate) {
		return ES0_PM_ERROR_NO_BAUDRATE;
	}

	if (hci_baudrate && ahi_baudrate && hci_baudrate != ahi_baudrate) {
		return ES0_PM_ERROR_BAUDRATE_MISMATCH;
	}

	used_baudrate = hci_baudrate ? hci_baudrate : ahi_baudrate;

	if (es0_user_counter) {
		/* Already started */
		es0_user_counter++;
		return ES0_PM_ERROR_NO_ERROR;
	}

	static uint8_t ll_boot_params_buffer[LL_BOOT_PARAMS_MAX_SIZE];

	memset(ll_boot_params_buffer, 0xFF, LL_BOOT_PARAMS_MAX_SIZE);
	uint8_t *ptr = ll_boot_params_buffer;
	*ptr++ = 'N';
	*ptr++ = 'V';
	*ptr++ = 'D';
	*ptr++ = 'S';

	uint16_t total_length = 4; /* N,V,D,S */

	total_length += add_nvds_param_length(BOOT_PARAM_LEN_LE_CODED_PHY_500);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_DFT_SLAVE_MD);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_CH_CLASS_REP_INTV);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_BD_ADDRESS);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_ACTIVITY_MOVE_CONFIG);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_SCAN_EXT_ADV);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_RSSI_THR);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_RSSI_THR);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_SLEEP_ENABLE);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_EXT_WAKEUP_ENABLE);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_ENABLE_CHANNEL_ASSESSMENT);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_RSSI_THR);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_UART_BAUDRATE);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_UART_INPUT_CLK_FREQ);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_EXT_WAKEUP_TIME);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_OSC_WAKEUP_TIME);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_RM_WAKEUP_TIME);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_EXT_WARMBOOT_WAKEUP_TIME);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_LPCLK_DRIFT);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_ACTCLK_DRIFT);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_CONFIGURATION);
	total_length += add_nvds_param_length(BOOT_PARAM_LEN_CONFIGURATION2);

	if (total_length > LL_BOOT_PARAMS_MAX_SIZE) {
		return ES0_PM_ERROR_TOO_MANY_BOOT_PARAMS;
	}

	uint8_t bd_address[BOOT_PARAM_LEN_BD_ADDRESS];
	uint32_t config = es0_start_params.hpa_enabled ? CONFIGURATION_RF_TYPE_HPA : 0;
	uint32_t edge_config = 0;

	if (IS_ENABLED(CONFIG_SOC_AB1C1F1M41820HH0) || IS_ENABLED(CONFIG_SOC_AB1C1F4M51820HH0)) {
		config |= CONFIGURATION_SOC_TYPE_CSP;
	}

	/* Only use the limit when it is not the maximum */
	if (CONFIG_ALIF_TX_MAX_POWER < CONFIG_ALIF_TX_MAX) {
		config |= CONFIGURATION_LIMIT_TX_POWER;
		config |= ((CONFIG_ALIF_TX_MAX_POWER << 4) & 0xff0);
	}

	if (IS_ENABLED(CONFIG_ALIF_EDGE_LIMIT_ON)) {
		edge_config = 0x01;
		edge_config |= (CONFIG_ALIF_EDGE_CHANNEL_POWER_LIMIT << 8);
		edge_config |= ((CONFIG_ALIF_EDGE_LOW_CHANNEL + 1) << 16);
		edge_config |= ((CONFIG_ALIF_EDGE_HIGH_CHANNEL + 1) << 24);
	}

	alif_eui48_read(bd_address);

	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_LE_CODED_PHY_500, CONFIG_ALIF_PM_LE_CODED_PHY_500,
			    BOOT_PARAM_LEN_LE_CODED_PHY_500);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_DFT_SLAVE_MD, CONFIG_ALIF_PM_DFT_SLAVE_MD,
			    BOOT_PARAM_LEN_DFT_SLAVE_MD);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_CH_CLASS_REP_INTV, CONFIG_ALIF_PM_CH_CLASS_REP_INTV,
			    BOOT_PARAM_LEN_CH_CLASS_REP_INTV);
	ptr = write_tlv_str(ptr, BOOT_PARAM_ID_BD_ADDRESS, bdaddr_reverse(bd_address),
			    BOOT_PARAM_LEN_BD_ADDRESS);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_ACTIVITY_MOVE_CONFIG,
			    CONFIG_ALIF_PM_ACTIVITY_MOVE_CONFIG,
			    BOOT_PARAM_LEN_ACTIVITY_MOVE_CONFIG);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_SCAN_EXT_ADV, CONFIG_ALIF_PM_SCAN_EXT_ADV,
			    BOOT_PARAM_LEN_SCAN_EXT_ADV);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_RSSI_HIGH_THR, CONFIG_ALIF_PM_RSSI_HIGH_THR,
			    BOOT_PARAM_LEN_RSSI_THR);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_RSSI_LOW_THR, CONFIG_ALIF_PM_RSSI_LOW_THR,
			    BOOT_PARAM_LEN_RSSI_THR);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_SLEEP_ENABLE, CONFIG_ALIF_PM_SLEEP_ENABLE,
			    BOOT_PARAM_LEN_SLEEP_ENABLE);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_EXT_WAKEUP_ENABLE, CONFIG_ALIF_PM_EXT_WAKEUP_ENABLE,
			    BOOT_PARAM_LEN_EXT_WAKEUP_ENABLE);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_ENABLE_CHANNEL_ASSESSMENT,
			    CONFIG_ALIF_PM_ENABLE_CH_ASSESSMENT,
			    BOOT_PARAM_LEN_ENABLE_CHANNEL_ASSESSMENT);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_RSSI_INTERF_THR, CONFIG_ALIF_PM_RSSI_INTERF_THR,
			    BOOT_PARAM_LEN_RSSI_THR);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_UART_BAUDRATE, used_baudrate,
			    BOOT_PARAM_LEN_UART_BAUDRATE);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_EXT_WAKEUP_TIME, CONFIG_ALIF_EXT_WAKEUP_TIME,
			    BOOT_PARAM_LEN_EXT_WAKEUP_TIME);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_OSC_WAKEUP_TIME, CONFIG_ALIF_OSC_WAKEUP_TIME,
			    BOOT_PARAM_LEN_OSC_WAKEUP_TIME);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_RM_WAKEUP_TIME, CONFIG_ALIF_RM_WAKEUP_TIME,
			    BOOT_PARAM_LEN_RM_WAKEUP_TIME);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_EXT_WARMBOOT_WAKEUP_TIME,
			    CONFIG_ALIF_EXT_WARMBOOT_WAKEUP_TIME,
			    BOOT_PARAM_LEN_EXT_WARMBOOT_WAKEUP_TIME);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_LPCLK_DRIFT, CONFIG_ALIF_MAX_SLEEP_CLOCK_DRIFT,
			    BOOT_PARAM_LEN_LPCLK_DRIFT);
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_ACTCLK_DRIFT, CONFIG_ALIF_MAX_ACTIVE_CLOCK_DRIFT,
			    BOOT_PARAM_LEN_ACTCLK_DRIFT);


	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_CONFIGURATION, config,
			    BOOT_PARAM_LEN_CONFIGURATION);

	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_CONFIGURATION2, edge_config,
			    BOOT_PARAM_LEN_CONFIGURATION2);


	uint32_t min_uart_clk_freq = used_baudrate * 16;
	uint32_t reg_uart_clk_cfg = LL_UART_CLK_SEL_CTRL_16MHZ;
	uint32_t ll_uart_clk_freq = 16000000;
	uint32_t es0_clock_select = CONFIG_SE_SERVICE_RF_CORE_FREQUENCY;

	/* UART input clock can be configured as 16/24/48Mhz */
	if (min_uart_clk_freq > 16000000) {
		if (min_uart_clk_freq <= 24000000) {
			ll_uart_clk_freq = 24000000;
			reg_uart_clk_cfg = LL_UART_CLK_SEL_CTRL_24MHZ;
		} else {
			ll_uart_clk_freq = 48000000;
			reg_uart_clk_cfg = LL_UART_CLK_SEL_CTRL_48MHZ;
		}
	}
	/* Add UART clock select */
	es0_clock_select |= reg_uart_clk_cfg;
	ptr = write_tlv_int(ptr, BOOT_PARAM_ID_UART_INPUT_CLK_FREQ, ll_uart_clk_freq,
			    BOOT_PARAM_LEN_UART_INPUT_CLK_FREQ);

	if (total_length < (LL_BOOT_PARAMS_MAX_SIZE - 2)) {
		ptr = write_tlv_int(ptr, BOOT_PARAM_ID_NO_PARAM, 0, 0);
		total_length += 3;
	}

	if (total_length != (ptr - ll_boot_params_buffer)) {
		return ES0_PM_ERROR_INVALID_BOOT_PARAMS;
	}

	return take_es0_into_use_with_params(ll_boot_params_buffer, total_length,
		es0_clock_select, es0_start_params.hpa_enabled);
}



int8_t stop_using_es0(void)
{
	if (!es0_user_counter) {
		return -1;
	}
	es0_user_counter--;
	if (!es0_user_counter) {
		if (se_service_shutdown_es0()) {
			return -2;
		}
	}

	return 0;
}

void wake_es0(const struct device *uart_dev)
{
	/* Init default to 2 which not affect anythong  0 & 1 are only possible values */
	uint32_t rts = 2, cts = 2;

	/* Read RTS and CTS line */
	if (uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_RTS, &rts)) {
		/* Line read not supported */
		return;
	}
	if (uart_line_ctrl_get(uart_dev, UART_LINE_MODEM_CTS, &cts)) {
		/* Line read not supported */
		return;
	}

	if (cts == 0) {
		/* Wakeup Riscv by low CTS line */
		if (rts == 1) {
			uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_RTS, 0);
			k_busy_wait(50);
		}
		uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_RTS, 1);
		uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_AFCE, 1);
	} else if (rts == 0) {
		/* Init Device state */
		uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_RTS, 1);
		uart_line_ctrl_set(uart_dev, UART_LINE_CTRL_AFCE, 1);
	}
}
