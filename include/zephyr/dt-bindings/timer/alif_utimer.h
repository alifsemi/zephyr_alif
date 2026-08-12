/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_ALIF_UTIMER_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_ALIF_UTIMER_H_

/* Timer counter direction macros */
#define ALIF_UTIMER_COUNTER_DIRECTION_UP       0x00000000U  /* Counter Sawtooth Up direction */
#define ALIF_UTIMER_COUNTER_DIRECTION_DOWN     0x00000010U  /* Counter Sawtooth Down direction */
#define ALIF_UTIMER_COUNTER_DIRECTION_TRIANGLE 0x00000020U  /* Counter Triangle type */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_ALIF_UTIMER_H_ */
