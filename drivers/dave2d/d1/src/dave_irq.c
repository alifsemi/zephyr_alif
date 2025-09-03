/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*-----------------------------------------------------------------------------
 * Project:     D/AVE
 * File:        dave_irq.c
 * Description: This file defines the D/AVE driver IRQ setting functions
 *---------------------------------------------------------------------------
 */

/*-----------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------
 */
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include "dave_base_intern.h"
#include "dave_base.h"
#include "dave_registermap.h"

#define D1_IRQCTL_ENABLE                                                                           \
	(D2IRQCTL_CLR_FINISH_DLIST | D2IRQCTL_CLR_FINISH_ENUM | D2IRQCTL_ENABLE_FINISH_DLIST)
#define D1_IRQCTL_CLEAR   D1_IRQCTL_ENABLE
#define D1_IRQCTL_DISABLE (D2IRQCTL_CLR_FINISH_DLIST | D2IRQCTL_CLR_FINISH_ENUM)

#define GPU2D_IRQ_irq      DT_IRQ(DAVE2D_DEV, irq)
#define GPU2D_IRQ_priority DT_IRQ(DAVE2D_DEV, priority)

static d1_device_intern *s_isr_context;

static struct k_sem irqSem;

void GPU2D_IRQHandler(void *arg);

/*--------------------------------------------------------------------------*/
void d1_irq_enable(void)
{
	/* Clear all interrupts and enable DLIST IRQ */
	D1_REG(D2_IRQCTL) = D1_IRQCTL_ENABLE;

	/* Set DAVE2D interrupt handler */
	IRQ_CONNECT(GPU2D_IRQ_irq, GPU2D_IRQ_priority, GPU2D_IRQHandler, NULL, 0)

	/* Enable DAVE2D interrupt */
	irq_enable(GPU2D_IRQ_irq);
}

/*--------------------------------------------------------------------------*/
void d1_irq_disable(void)
{
	/* Clear all interrupts and disable DLIST IRQ */
	D1_REG(D2_IRQCTL) = D1_IRQCTL_DISABLE;

	/* Disable D/AVE2D interrupt */
	irq_disable(GPU2D_IRQ_irq);
}

/*--------------------------------------------------------------------------*/
int d1_queryirq(d1_device *handle, int irqmask, int timeout)
{
	(void)handle;
	(void)irqmask;

	if (timeout == d1_to_no_wait) {
		return 1;
	}

	if (k_sem_take(&irqSem, K_MSEC(timeout))) {
		return GPU2D_IRQ_irq;
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
void d1_set_isr_context(void *context)
{
	s_isr_context = (d1_device_intern *)context;

	if (context != NULL) {
		k_sem_init(&irqSem, 0, 1);
	}
}

/*--------------------------------------------------------------------------*/
void GPU2D_IRQHandler(void *arg)
{
	(void)arg;

	/* Get D/AVE2D interrupt status */
	long dave_status = D1_REG(D2_STATUS);

	/* Clear all interrupts triggered */
	D1_REG(D2_IRQCTL) = D1_IRQCTL_CLEAR;

	/* Check if DLIST IRQ triggered */
	if (dave_status & D2C_IRQ_DLIST) {
#ifdef CONFIG_D1_DLIST_INDIRECT
		if (s_isr_context && s_isr_context->dlist_indirect &&
		    ((void *)*s_isr_context->dlist_start) != NULL) {
			D1_REG(D2_DLISTSTART) = *s_isr_context->dlist_start;

			++s_isr_context->dlist_start;
		}
#endif

		k_sem_give(&irqSem);
	}
}
