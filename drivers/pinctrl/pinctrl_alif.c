/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_pinctrl

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/cpu.h>

/* Pinmux settings:
 *   See soc/alif/common/pinctrl_soc.h for definitions.
 */

int pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t pinmux_value = *(uint32_t *)pin;
	mem_addr_t pinctrl_addr;
	uint32_t pinctrl_data;

	/* is port LPGPIO? */
	if (LPGPIO_PINCTRL_BASE && (GET_PINMUX_PORT(pinmux_value) == LPGPIO_PORT)) {
		/* get the address of the LPGPIO pin using port value */
		pinctrl_addr = (mem_addr_t)LPGPIO_PINMUX_ADDR(pinmux_value);

		/* get LPGPIO port pin-pad value. */
		pinctrl_data = ((pinmux_value >> LPGPIO_PIN_PAD_SHIFT) & LPGPIO_PIN_PAD_MASK);
	} else {
		/* get the address of the pin using port value */
		pinctrl_addr = (mem_addr_t)PINMUX_ADDR(pinmux_value);

		/* clear only the port value, other fields are set already */
		pinctrl_data = (pinmux_value & (~PIN_NUM_CLR_MASK));
	}

	/* set the pinmux and pin-pad value. */
	sys_write32(pinctrl_data, pinctrl_addr);

	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins,
			   uint8_t pin_cnt, uintptr_t reg)
{
	int ret = 0;

	ARG_UNUSED(reg);
	while (pin_cnt-- > 0) {
		ret = pinctrl_configure_pin(pins++);
		if (ret < 0) {
			break;
		}
	}
	return ret;
}
