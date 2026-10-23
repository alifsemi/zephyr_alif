/*
 * Copyright (c) 2022 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_SNPS_DESIGNWARE_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_SNPS_DESIGNWARE_GPIO_H_

/**
 * @brief Enable GPIO pin debounce.
 *
 * The debounce flag is a Zephyr specific extension of the standard GPIO flags
 * specified by the Linux GPIO binding. Only applicable for SNPS DesignWare GPIO
 * controllers.
 */
#define DW_GPIO_DEBOUNCE (1U << 8)

/**
 * @brief Mark pin as a DMA trigger source.
 *
 * INTEN/type/polarity/debounce are programmed as for a GPIO interrupt.
 * INTMASK is left set so the CPU ISR does not EOI the edge that
 * GPIOx_n_DMA_REQ shares.
 *
 * Call gpio_pin_configure() with this flag before
 * gpio_pin_interrupt_configure(). To use the pin as a CPU IRQ later,
 * gpio_pin_configure() again without this flag, then interrupt configure.
 */
#define DW_GPIO_DMA_TRIG (1U << 9)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_SNPS_DESIGNWARE_GPIO_H_ */
