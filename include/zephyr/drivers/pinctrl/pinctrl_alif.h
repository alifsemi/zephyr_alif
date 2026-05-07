/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PINCTRL_ALIF_H_
#define ZEPHYR_INCLUDE_DRIVERS_PINCTRL_ALIF_H_

/*
 * Alif Ensemble pinmux encoding:
 *   bits 0:2  = function select (0-7)
 *   bits 3:9  = absolute pin number (bits 3:10 on Gen2)
 *   bit 16    = read enable
 *   bit 17    = schmitt trigger enable
 *   bit 18    = slew rate
 *   bits 19:20 = driver disabled state control
 *   bits 21:22 = output drive strength
 *   bit 23    = driver type (push-pull / open-drain)
 */

#define ALIF_PINMUX_FUNC_SHIFT  3

#if defined(CONFIG_ENSEMBLE_GEN2)
#define ALIF_PINMUX_PIN_MASK    0xFF
#define ALIF_PINMUX_CLR_MASK    0x7F8
#else
#define ALIF_PINMUX_PIN_MASK    0x7F
#define ALIF_PINMUX_CLR_MASK    0x3F8
#endif

#define ALIF_PINMUX_GET_PIN(pmux) \
	(((pmux) >> ALIF_PINMUX_FUNC_SHIFT) & ALIF_PINMUX_PIN_MASK)

#endif /* ZEPHYR_INCLUDE_DRIVERS_PINCTRL_ALIF_H_ */
