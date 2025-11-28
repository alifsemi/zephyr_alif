/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/drivers/hwinfo.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <soc_common.h>
#include "se_service.h"

#if defined(CONFIG_RTSS_HP)
#define RESET_STATUS_REG            (AON_RTSS_HP_RESET)
#elif defined(CONFIG_RTSS_HE)
#define RESET_STATUS_REG            (AON_RTSS_HE_RESET)
#else
#error "Invalid CPU"
#endif

#define DEV_SERIAL_NUM_SIZE                     8
#define POR_OR_SESS_RESET                       BIT(0)
#define NSRST                                   BIT(1)
#define HOST_SYS_RESET_REQ_BY_SESS              BIT(3)
#define RESET_REQ_TO_THE_POWER_DOMAIN           BIT(4)

static get_device_revision_data_t dev_data;
static uint8_t dev_serial_num[DEV_SERIAL_NUM_SIZE] = {0};

/**
 * @brief format_contents - converts numbers in src into
 * characters for displaying.
 * parameters,
 * dst - placeholder containing numbers converted to characters.
 * src - placeholder containing numbers to be converted.
 * bytes - length of numbers to be converted.
 */
static void format_contents(uint8_t *dst, const uint8_t *src, int bytes)
{
	uint8_t digit_len = 0;
	int i = 0, pos = 0;

	for (i = 0; i < bytes; ++i) {
		/* pass digit length as 2 as two characters need to */
		/* be printed if the number is more than 0xF */
		if (src[i] > 0xF) {
			digit_len = 2;
		} else {
			digit_len = 1;
		}

		/* extra 1 for '\0' */
		snprintk(&dst[pos], digit_len + 1, "%x", src[i]);
		pos += digit_len;
	}
}

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	int ret;

	/* Input validation */
	if (!buffer || length == 0) {
		return -EINVAL;
	}

	/* Get device data from Secure Enclave */
	ret = se_service_system_get_device_data(&dev_data);
	if (ret) {
		return ret;
	}

	format_contents(&dev_serial_num[0],
				(uint8_t *)&dev_data.SerialN[0], sizeof(dev_data.SerialN));
	if (length > DEV_SERIAL_NUM_SIZE) {
		length = DEV_SERIAL_NUM_SIZE;
	}
	memcpy(buffer, dev_serial_num, length);

	return length;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t flags = 0;
	uint32_t reason = sys_read32(RESET_STATUS_REG);

	if (reason & POR_OR_SESS_RESET) {
		flags |= RESET_POR;
	}
	if (reason & NSRST) {
		flags |= RESET_PIN;
	}
	if (reason & HOST_SYS_RESET_REQ_BY_SESS) {
		flags |= RESET_SOFTWARE;
	}
	if (reason & RESET_REQ_TO_THE_POWER_DOMAIN) {
		flags |= RESET_SOFTWARE;
	}
	*cause = flags;

	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	/* Read the set bits */
	uint32_t reason = sys_read32(RESET_STATUS_REG);

	/* Write them back to clear the reset */
	sys_write32(reason, RESET_STATUS_REG);

	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = (RESET_POR | RESET_PIN | RESET_SOFTWARE);

	return 0;
}
