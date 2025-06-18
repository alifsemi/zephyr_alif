/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <stdio.h>

int main(void)
{
#define OSC_CLOCK_SRC_FREQ(node)     DT_PROP(DT_PATH(clocks, node), clock_frequency)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lfxo), okay)
#define ALIF_CLOCK_S32K_CLK_FREQ OSC_CLOCK_SRC_FREQ(lfxo)
#else
#define ALIF_CLOCK_S32K_CLK_FREQ OSC_CLOCK_SRC_FREQ(lfrc)
#endif

	printf("s32k %d\n", ALIF_CLOCK_S32K_CLK_FREQ);
//	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	return 0;
}
