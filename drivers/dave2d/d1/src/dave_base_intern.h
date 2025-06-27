/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __1_dave_base_intern_h_H
#define __1_dave_base_intern_h_H

/*---------------------------------------------------------------------------
 *  Section: Macros
 */

#include <zephyr/device.h>

/* Define the D/AVE2D device node */
#define DAVE2D_DEV DT_ALIAS(d2_inst)

/* Define the D/AVE2D base address */
#define GPU2D_BASE DT_REG_ADDR(DAVE2D_DEV)

/* Get D/AVE2D register */
#define D1_REG(index) ((long *)GPU2D_BASE)[index]

/* Convert d1_device */
#define D1_DEV(handle) ((d1_device_intern *)handle)

/*---------------------------------------------------------------------------
 *  Section: Types and enumerations
 */

/*--------------------------------------------------------------------------
 *  Type: struct d1_device_intern
 *
 *  Internal D/AVE2D device handle
 *
 *  Members:
 *    dlist_indirect - flag to enable indirect display lists
 *    dlist_start    - dlist start addresses, end is marked with 0
 *
 */
typedef struct _d1_device_intern {
	long flags;
	int dlist_indirect;
	volatile long *dlist_start;
} d1_device_intern;

#endif
