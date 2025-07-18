/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*-----------------------------------------------------------------------------
 * Project:     D/AVE
 * File:        dave_base.c
 * Description: This file defines the D/AVE low-level driver basic functions
 *---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "dave_base.h"
#include "dave_base_intern.h"
#include "dave_registermap.h"
#include "dave_d0lib.h"
#include "dave_irq.h"
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/alif_ensemble_clocks.h>

/*---------------------------------------------------------------------------
 * These helper macros are used to stringify a given macro
 */
#define D1_STR(s)  #s
#define D1_XSTR(s) D1_STR(s)

/* These helper macros are used to concat two strings with a dot. */
#ifdef __CA850__
#define D1_DOT(a, b) a.b
#else
#define D1_DOT(a, b) a##.##b
#endif
#define D1_XDOT(a, b) D1_DOT(a, b)

/*---------------------------------------------------------------------------
 * Define the D1_VERSION and D1_VERSION_STRING macros
 */

/* Build up the D1_VERSION macro */
#define D1_VERSION ((D1_VERSION_MAJOR << 16) | D1_VERSION_MINOR)

/* Create the D1_VERSION_STRING macro */
#define D1_VERSION_STRING D1_XSTR(D1_XDOT(D1_VERSION_MAJOR, D1_VERSION_MINOR))

/* Define DAVE2D clock */
#define DAVE2D_CLK DT_CLOCKS_CELL(DAVE2D_DEV, clkid)

/*--------------------------------------------------------------------------*/

static const char g_versionid[] = "V" D1_VERSION_STRING;

static const struct device *const clock_ctrl = DEVICE_DT_GET(DT_NODELABEL(clock));

/*--------------------------------------------------------------------------*/
const char *d1_getversionstring(void)
{
	return g_versionid;
}

/*--------------------------------------------------------------------------*/
int d1_getversion(void)
{
	return D1_VERSION;
}

/*--------------------------------------------------------------------------*/
static void d1_clock_enable(void)
{
	if (!device_is_ready(clock_ctrl)) {
		return;
	}

	clock_control_on(clock_ctrl, (clock_control_subsys_t)DAVE2D_CLK);
}

/*--------------------------------------------------------------------------*/
static void d1_clock_disable(void)
{
	if (!device_is_ready(clock_ctrl)) {
		return;
	}

	clock_control_on(clock_ctrl, (clock_control_subsys_t)DAVE2D_CLK);
}

/*--------------------------------------------------------------------------*/
d1_device *d1_opendevice(long flags)
{
	d1_device *handle = d1_allocmem(sizeof(d1_device_intern));

	if (handle == NULL) {
		return NULL;
	}

	D1_DEV(handle)->flags = flags;

#ifdef CONFIG_D1_DLIST_INDIRECT
	D1_DEV(handle)->dlist_indirect = 1;
#endif

	/* Enable D/AVE2D clock */
	d1_clock_enable();

	/* Enable interrupts */
	d1_irq_enable();
	d1_set_isr_context(handle);

	return handle;
}

/*--------------------------------------------------------------------------*/
int d1_closedevice(d1_device *handle)
{
	/* Disable interrupts */
	d1_irq_disable();
	d1_set_isr_context(NULL);

	/* Disable clock */
	d1_clock_disable();

	d1_freemem(handle);

	return 1;
}

/*--------------------------------------------------------------------------*/
int d1_devicesupported(d1_device *handle, int deviceid)
{
	switch (deviceid) {
	case D1_DAVE2D:
#ifdef CONFIG_D1_DLIST_INDIRECT
	case D1_DLISTINDIRECT:
#endif
		return 1;

	default:
		return 0;
	}
}

/*--------------------------------------------------------------------------*/
long d1_getregister(d1_device *handle, int deviceid, int index)
{
	if (handle == NULL) {
		return 0;
	}

	switch (deviceid) {
	case D1_DAVE2D:
		return D1_REG(index);

#ifdef CONFIG_D1_DLIST_INDIRECT
	case D1_DLISTINDIRECT:
		return D1_DEV(handle)->dlist_indirect;
#endif

	default:
		return 0;
	}
}

/*--------------------------------------------------------------------------*/
void d1_setregister(d1_device *handle, int deviceid, int index, long value)
{
	if (handle == NULL) {
		return;
	}

	switch (deviceid) {
	case D1_DAVE2D:
#ifdef CONFIG_D1_DLIST_INDIRECT
		if (index == D2_DLISTSTART && D1_DEV(handle)->dlist_indirect) {
			long *dlist_ptr = (long *)value;

			D1_DEV(handle)->dlist_start = dlist_ptr + 1;
			D1_REG(index) = *dlist_ptr;
		} else {
			D1_REG(index) = value;
		}
#else
		D1_REG(index) = value;
#endif

		break;

#ifdef CONFIG_D1_DLIST_INDIRECT
	case D1_DLISTINDIRECT:
		D1_DEV(handle)->dlist_indirect = value;
#endif

	default:
		break;
	}
}
