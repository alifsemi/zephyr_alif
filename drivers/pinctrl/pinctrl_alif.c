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
 * syntax : U  U  U  U  U  U  P  P  P  P  P  P  P  F  F  F
 * bit pos: 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
 *
 * syntax : U  U  U  U  U  U  U  U  DR E2 E1 P2 P1 SR ST REN
 * bit pos: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *
 * bits 0:2 [FFF] denote pinmux functions
 * bits 3:9 [PPPPPPP] denote port values
 * bits 11:15 and 24:31 are unused
 * bit 16 denotes read enable
 * bit 17 denotes schmitt trigger enable
 * bit 18 denotes slew rate
 * bits 19:20 denote driver disabled state control
 * bits 21:22 denote output drive strength
 * bit 23 denotes driver type
 *
 * LPGPIO Pinmux settings:
 * syntax : U  U  U  U  U  U  U  U  DR E2 E1 P2 P1 SR ST REN
 * bit pos: 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
 *
 * syntax : U  U  U  U  U  U  U  U  U  U  U  U  U  U  U  U
 * bit pos: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 *
 * bit 0 denotes read enable
 * bit 1 denotes schmitt trigger enable
 * bit 2 denotes slew rate
 * bits 3:4 denote driver disabled state control
 * bits 5:6 denote output drive strength
 * bit 7 denotes driver type
 * bits 8:31 are unused
 */
#define PIN_FUNC_SHIFT 3
#if defined(CONFIG_SOC_SERIES_ENSEMBLE_E8) || defined(CONFIG_SOC_SERIES_ENSEMBLE_E4)
/* bits 3:10 [PPPPPPPP] denote port values */
#define PIN_NUM_MASK 0xFF
#else
#define PIN_NUM_MASK 0x7F
#endif
#define PIN_NUM_CLR_MASK 0x3F8

#define PORT_P15             120
#define LPGPIO_PORT          PORT_P15
#define LPGPIO_PIN_NUM_MASK  0x7
#define LPGPIO_PIN_PAD_SHIFT 16
#define LPGPIO_PIN_PAD_MASK  0xFF

#if DT_NODE_HAS_PROP(DT_NODELABEL(pinctrl), reg)
#define PINCTRL_BASE         DT_REG_ADDR(DT_NODELABEL(pinctrl))
#define LPGPIO_PINCTRL_BASE  COND_CODE_1(DT_REG_HAS_IDX(DT_NODELABEL(pinctrl), 1), \
				(DT_REG_ADDR_BY_NAME(DT_NODELABEL(pinctrl), lpgpio_pinctrl)), \
				(0))
#endif

#define GET_PINMUX_PORT(value) ((value >> PIN_FUNC_SHIFT) & PIN_NUM_MASK)

#define PINMUX_ADDR(value) (PINCTRL_BASE + \
		(((value >> PIN_FUNC_SHIFT) & PIN_NUM_MASK) * 4))

#define LPGPIO_PINMUX_ADDR(value) (LPGPIO_PINCTRL_BASE + \
		((value & LPGPIO_PIN_NUM_MASK) * 4))

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
