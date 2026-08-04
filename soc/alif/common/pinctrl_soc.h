/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ALIF_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ALIF_COMMON_PINCTRL_SOC_H_

#include <stdint.h>
#include <zephyr/devicetree.h>

typedef uint32_t pinctrl_soc_pin_t;

#define Z_PINCTRL_ALIF_PINCFG(node_id)					\
	(PAD_CONF_REN(DT_ENUM_IDX(node_id, read_enable)) |		\
	PAD_CONF_SMT(DT_ENUM_IDX(node_id, schmitt_enable)) |		\
	PAD_CONF_SR(DT_ENUM_IDX(node_id, slew_rate)) |			\
	PAD_CONF_DSC(DT_ENUM_IDX(node_id, driver_state_control)) |	\
	PAD_CONF_ODS(DT_ENUM_IDX(node_id, drive_strength)) |		\
	PAD_CONF_DRV(DT_ENUM_IDX(node_id, driver)))

#define Z_PINCTRL_STATE_PIN_INIT(group, pin_prop, idx)			\
	DT_PROP_BY_IDX(group, pin_prop, idx) | Z_PINCTRL_ALIF_PINCFG(group),

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			\
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),		\
		DT_FOREACH_PROP_ELEM, pinmux, Z_PINCTRL_STATE_PIN_INIT)};

#define ONE_BIT_FIELD_MASK	0x1
#define TWO_BIT_FIELD_MASK	0x3

/* Pinmux settings:
 * syntax : U  U  U  U  U  U  P  P  P  P  P  P  P  F  F  F
 * bit pos: 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
 * syntax : U  U  U  U  U  U  U  U  DR E2 E1 P2 P1 SR ST REN
 * bit pos: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 * bits 0:2 [FFF] denote pinmux functions
 * bits 3:9 [PPPPPPP] denote port values
 * bits 11:15 and 24:31 are unused
 * bit 16 denotes read enable
 * bit 17 denotes schmitt trigger enable
 * bit 18 denotes slew rate
 * bits 19:20 denote driver disabled state control
 * bits 21:22 denote output drive strength
 * bit 23 denotes driver
 *
 * LPGPIO Pinmux settings:
 * syntax : U  U  U  U  U  U  U  U  DR E2 E1 P2 P1 SR ST REN
 * bit pos: 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
 * syntax : U  U  U  U  U  U  U  U  U  U  U  U  U  U  U  U
 * bit pos: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 * bit 0 denotes read enable
 * bit 1 denotes schmitt trigger enable
 * bit 2 denotes slew rate
 * bits 3:4 denote driver disabled state control
 * bits 5:6 denote output drive strength
 * bit 7 denotes driver type
 * bits 8:31 are unused
 */

#define REN_BIT_PST	    16
#define SMP_BIT_PST	    17
#define SR_BIT_PST	    18
#define DSC_BIT_PST	    19
#define ODS_BIT_PST	    21
#define DRV_BIT_PST	    23

#define DSC_HIGH_Z	    0
#define DSC_PULL_UP	    1
#define DSC_PULL_DOWN   2
#define DSC_BUS_REPEAT  3

#define GPIO_PORT_BASE_ADDRESS 0x49000000
/* LPGPIO has a base address of 0x42002000 and is port 15, all other ports are spaced
 * 0x1000 away from the base address in port order.
 */
#define GPIO_PORT_FROM_ADDRESS(x) \
	(x < GPIO_PORT_BASE_ADDRESS ? 15U : (x - GPIO_PORT_BASE_ADDRESS) / 0x1000)

#define PAD_CONF_REN(x) ((x & ONE_BIT_FIELD_MASK) << REN_BIT_PST)
#define PAD_CONF_SMT(x) ((x & ONE_BIT_FIELD_MASK) << SMP_BIT_PST)
#define PAD_CONF_SR(x)  ((x & ONE_BIT_FIELD_MASK) << SR_BIT_PST)
#define PAD_CONF_DSC(x) ((x & TWO_BIT_FIELD_MASK) << DSC_BIT_PST)
#define PAD_CONF_ODS(x) ((x & TWO_BIT_FIELD_MASK) << ODS_BIT_PST)
#define PAD_CONF_DRV(x) ((x & ONE_BIT_FIELD_MASK) << DRV_BIT_PST)

#define PIN_FUNC_SHIFT 3
#if defined(CONFIG_ENSEMBLE_GEN2)
/* bits 3:10 [PPPPPPPP] denote port values
 * For Ensemble Gen2, bits 3:10 of the pinmux value form an 8-bit field:
 *   - After shifting by PIN_FUNC_SHIFT (3), this field appears in bits 0:7.
 *   - PIN_NUM_MASK (0xFF) selects the full 8-bit field used to compute the
 *     pinmux register index/address.
 *   - PORT_NUM_MASK (0xF8) selects only bits 3:7 of that shifted value,
 *     i.e. a 5-bit logical port number.
 */
#define PIN_NUM_MASK 0xFF
#define PORT_NUM_MASK 0xF8
#else
#define PIN_NUM_MASK 0x7F
#define PORT_NUM_MASK 0x78
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

#define GET_PINMUX_PORT(value) ((value >> PIN_FUNC_SHIFT) & PORT_NUM_MASK)

#define PINMUX_ADDR(value) (PINCTRL_BASE + \
		(((value >> PIN_FUNC_SHIFT) & PIN_NUM_MASK) * 4))

#define LPGPIO_PINMUX_ADDR(value) (LPGPIO_PINCTRL_BASE + \
		((value & LPGPIO_PIN_NUM_MASK) * 4))

#endif /* ZEPHYR_SOC_ALIF_COMMON_PINCTRL_SOC_H_ */
