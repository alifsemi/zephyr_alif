/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include "ipm_arm_mhuv2.h"

#define DT_DRV_COMPAT	arm_mhuv2

#define BIT0_1		0x1
#define TIMEOUT		0xFFFF00

#define DRV_MHUV2_CFG(d)	((const struct mhuv2_device_config *)(d)->config)
#define DRV_MHUV2_DATA(d)	((struct mhuv2_device_data *)(d)->data)

/**
 * @fn      static int mhuv2_send(const struct device *dev, int wait, uint32_t ch_id,
 *				const void *pdata, int size)
 * @brief   Send data on MHUV2
 * @param[in] dev   : pointer to Runtime device structure
 * @param[in] wait  : time to wait for the data to be sent
 * @param[in] ch_id : channel identifier
 * @param[in] pdata : pointer to data
 * @param[in] size  : data size in bytes
 * @return  Execution status
 */
static int mhuv2_send(const struct device *dev, int wait, uint32_t ch_id,
		      const void *pdata, int size)
{
	struct mhuv2_device_data *data = DRV_MHUV2_DATA(dev);

	ARG_UNUSED(wait);
	ARG_UNUSED(size);

	if ((dev == NULL) || (pdata == NULL) || ch_id >= data->max_ch) {
		return -EINVAL;
	}

	struct MHUV2_SND *SND = (struct MHUV2_SND *)DEVICE_MMIO_GET(dev);
	uint32_t snd_timeout = TIMEOUT;

	/* Clear Interrupt Status */
	SND->INT_CLR = (NR2R_INTR | R2NR_INTR | CHCOMB_INTR);
	/* Clear Interrupt generation (NR2R Int, R2NR Int, Combined Int) */
	SND->INT_EN = SND->INT_EN & ~(NR2R_INTR | R2NR_INTR | CHCOMB_INTR);
	/* Enable Interrupt generation (NR2R Int, R2NR Int, Combined Int) */
	SND->INT_EN = (NR2R_INTR | R2NR_INTR | CHCOMB_INTR);

	/* Send Access Request */
	SND->ACCESS_REQUEST = 0x1U;

	if (!(SND->ACCESS_READY & BIT0_1)) {
		while (snd_timeout) {
			if ((SND->ACCESS_READY & BIT0_1)) {
				/* Receiver is ready */
				break;
			}
			--snd_timeout;
		}
		if (!snd_timeout) {
			/* Receiver is busy */
			SND->ACCESS_REQUEST = 0x00000000;
			return -EBUSY;
		}
	}

	/* Clear any pending channel interrupts */
	SND->CHANNEL[ch_id].CH_INT_CLR = BIT0_1;

	SND->CHANNEL[ch_id].CH_INT_EN = BIT0_1;
	SND->CHANNEL[ch_id].CH_SET = *((uint32_t *)pdata);

	return 0;
}

/**
 * @fn      static void mhuv2_register_cb(const struct device *dev,
 *				ipm_callback_t cb, void *user_data)
 * @brief   Register for MHUV2 data callback
 * @param[in] dev	: pointer to Runtime device structure
 * @param[in] cb	: pointer to callback function
 * @param[in] user_data : pointer to user data
 * @return  None
 */
static void mhuv2_register_cb(const struct device *dev,
		ipm_callback_t cb, void *user_data)
{
	struct mhuv2_device_data *mhuv2_cb_data = DRV_MHUV2_DATA(dev);

	mhuv2_cb_data->callback = cb;
	mhuv2_cb_data->user_data = (uint32_t *)user_data;
}

/**
 * @fn      static uint32_t mhuv2_max_ch_val_get(const struct device *dev)
 * @brief   Fetch the max channels supported
 * @param[in] dev : pointer to Runtime device structure
 * @return  Supported maximum MHU v2 channels
 */
static uint32_t mhuv2_max_ch_val_get(const struct device *dev)
{
	struct mhuv2_device_data *drv_data = DRV_MHUV2_DATA(dev);

	return drv_data->max_ch;
}

/**
 * @fn      static int mhuv2_set_enabled(const struct device *dev, int enable)
 * @brief   Enable receiver MHU v2 interrupt.
 *	    Shall be used only for receiver side MHU
 * @param[in] dev    : pointer to Runtime device structure
 * @param[in] enable : Flag for enabling mhu
 * @return  Execution status
 */
static int mhuv2_set_enabled(const struct device *dev, int enable)
{
	const struct mhuv2_device_config *config = DRV_MHUV2_CFG(dev);

	/* Return error if Tx MHU is referenced */
	if (config->irq_type) {
		return -ENOTSUP;
	}

	struct MHUV2_REC *REC = (struct MHUV2_REC *)DEVICE_MMIO_GET(dev);

	/* Clear Interrupt Status */
	REC->INT_CLR = (NR2R_INTR | R2NR_INTR | CHCOMB_INTR);
	/* Clear Interrupt generation (NR2R Int, R2NR Int, Combined Int) */
	REC->INT_EN = REC->INT_EN & ~(NR2R_INTR | R2NR_INTR | CHCOMB_INTR);

	if (enable) {
		/* Enable Interrupt generation (Combined CH Int) */
		REC->INT_EN = CHCOMB_INTR;
		/* Enable interrupt */
		irq_enable(config->irq_num);
	} else {
		/* Disable interrupt */
		irq_disable(config->irq_num);
	}
	return 0;
}

/**
 * @fn      static int mhuv2_init(const struct device *dev)
 * @brief   Initialise  MHU v2
 * @param[in] dev : pointer to Runtime device structure
 * @return  Execution status
 */
static int mhuv2_init(const struct device *dev)
{
	const struct mhuv2_device_config *config	= DRV_MHUV2_CFG(dev);
	struct mhuv2_device_data *data			= DRV_MHUV2_DATA(dev);

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	config->irq_init_func(dev);
	data->callback	= NULL;
	data->user_data	= NULL;

	if (config->irq_type) {
		struct MHUV2_SND *SND = (struct MHUV2_SND *)DEVICE_MMIO_GET(dev);
		uint8_t ch = 0;

		data->max_ch =
		(((struct MHUV2_SND *)DEVICE_MMIO_GET(dev))->MHU_CFG & MHU_CFG_DEF_Msk);
		/* Clear Interrupt generation (NR2R Int, R2NR Int, Combined Int) */
		SND->INT_EN = SND->INT_EN & ~(NR2R_INTR | R2NR_INTR | CHCOMB_INTR);
		/* Clear Interrupt Status */
		SND->INT_CLR = (NR2R_INTR | R2NR_INTR | CHCOMB_INTR);

		for (ch = 0; ch < data->max_ch; ch++) {
			/* Clear any pending channel interrupts */
			SND->CHANNEL[ch].CH_INT_EN &= ~(BIT0_1);
			SND->CHANNEL[ch].CH_INT_CLR = BIT0_1;
		}

		/* Enable interrupt */
		irq_enable(config->irq_num);
	} else {
		data->max_ch =
		(((struct MHUV2_REC *)DEVICE_MMIO_GET(dev))->MHU_CFG & MHU_CFG_DEF_Msk);
	}

	return 0;
}

/**
 * @fn      static void mhuv2_send_isr(const struct device *dev)
 * @brief   Sender MHU v2's interrupt service routine
 * @param[in] dev : pointer to Runtime device structure
 * @return  None
 */
static void mhuv2_send_isr(const struct device *dev)
{
	struct MHUV2_SND *SND			= (struct MHUV2_SND *)DEVICE_MMIO_GET(dev);
	uint32_t snd_int_st			= SND->INT_ST;
	struct mhuv2_device_data *drv_data	= DRV_MHUV2_DATA(dev);
	uint8_t ch_id;

	/* Check NR2R interrupt Status */
	if ((snd_int_st & NR2R_INTR) == NR2R_INTR) {
		SND->INT_CLR = NR2R_INTR;
	}
	/* Check R2NR interrupt status */
	if ((snd_int_st & R2NR_INTR) == R2NR_INTR) {
		SND->INT_CLR = R2NR_INTR;
	}
	/* Check Combined interrupt status */
	if (((snd_int_st & CHCOMB_INTR) == CHCOMB_INTR) && (SND->CH_INT_ST0 != 0x0)) {
		for (ch_id = 0 ; ch_id < drv_data->max_ch; ++ch_id) {
			if ((SND->CHANNEL[ch_id].CH_INT_ST & BIT0_1)
				== BIT0_1) {
				/* Ack the CHCOMB interrupt */
				SND->CHANNEL[ch_id].CH_INT_CLR = BIT0_1;
				/* Channel's CH_INT_EN makes sure */
				/* the interrupt is generated on */
				/* setting CH_SET. */
				/* CH_ST is checked to make sure */
				/* receiver got the data and */
				/* set ACC_REQ to 0. */
				if ((SND->CHANNEL[ch_id].CH_ST == 0) &&
				    (SND->CHANNEL[ch_id].CH_INT_EN & BIT0_1)) {
					SND->CHANNEL[ch_id].CH_INT_EN
							&= ~(BIT0_1);
					SND->ACCESS_REQUEST = 0x0U;
					if (drv_data->callback)
						drv_data->callback(dev, drv_data->user_data,
								ch_id, NULL);
				}
			}
		}
	}
}

/**
 * @fn      static void mhuv2_recv_isr(const struct device *dev)
 * @brief   Recriver MHU v2's interrupt service routine
 * @param[in] dev : pointer to Runtime device structure
 * @return  None
 */
static void mhuv2_recv_isr(const struct device *dev)
{
	struct mhuv2_device_data *drv_data	= DRV_MHUV2_DATA(dev);
	uint8_t offset				= 0;
	volatile uint32_t recv_ch_status_reg;
	uint8_t ch_id;

	struct MHUV2_REC *RECV = (struct MHUV2_REC *)DEVICE_MMIO_GET(dev);

	for (ch_id = 0; ch_id < drv_data->max_ch; ch_id++) {
		if (ch_id >= CHCOMB_INT_ST0_BEGIN &&
		    ch_id <= CHCOMB_INT_ST0_END) {
			offset = CHCOMB_INT_ST0_BEGIN;
			recv_ch_status_reg = RECV->CH_INT_ST0;
		} else if (ch_id >= CHCOMB_INT_ST1_BEGIN &&
			ch_id <= CHCOMB_INT_ST1_END) {
			offset = CHCOMB_INT_ST0_END;
			recv_ch_status_reg = RECV->CH_INT_ST1;
		} else if (ch_id >= CHCOMB_INT_ST2_BEGIN &&
			ch_id <= CHCOMB_INT_ST2_END) {
			offset = CHCOMB_INT_ST1_END;
			recv_ch_status_reg = RECV->CH_INT_ST2;
		} else if (ch_id >= CHCOMB_INT_ST3_BEGIN &&
			ch_id <= CHCOMB_INT_ST3_END) {
			offset = CHCOMB_INT_ST2_END;
			recv_ch_status_reg = RECV->CH_INT_ST3;
		}
		if ((recv_ch_status_reg) & (BIT0_1 << (ch_id - offset))) {
			*(drv_data->user_data) = RECV->CHANNEL[ch_id].CH_ST;
			if (drv_data->callback)
				drv_data->callback(dev, drv_data->user_data, ch_id, NULL);
			RECV->CHANNEL[ch_id].CH_CLR = 0xFFFFFFFF;
		}
	}
}

/**
 * @fn      static void  mhuv2_isr(const struct device *dev)
 * @brief   MHU v2's interrupt service routine
 * @param[in] dev : pointer to Runtime device structure
 * @return  None
 */
static void mhuv2_isr(const struct device *dev)
{
	const struct mhuv2_device_config *config = DRV_MHUV2_CFG(dev);

	if (config->irq_type) {
		mhuv2_send_isr(dev);
	} else {
		mhuv2_recv_isr(dev);
	}
}

static struct ipm_driver_api mhuv2_driver_api = {
	.send			= mhuv2_send,
	.register_callback	= mhuv2_register_cb,
	.max_data_size_get	= NULL,
	.max_id_val_get		= mhuv2_max_ch_val_get,
	.set_enabled		= mhuv2_set_enabled,
#ifdef CONFIG_IPM_CALLBACK_ASYNC
	.complete		= NULL
#endif
};

#if defined(CONFIG_ARM_MHUV2)
#define ARM_MHUV2_DATA_INST(n)						\
	static void mhuv2_##n##_irq_init(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    0,						\
			    mhuv2_isr,					\
			    DEVICE_DT_INST_GET(n), 0);			\
	}								\
	static const struct mhuv2_device_config mhuv2_cfg_##n = {	\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),			\
		.irq_num = DT_INST_IRQN(n),				\
		.irq_init_func = mhuv2_##n##_irq_init,			\
		.irq_type = DT_INST_IRQ_HAS_NAME(n, tx),		\
	};								\
	static struct mhuv2_device_data mhuv2_data_##n;			\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			      mhuv2_init,				\
			      NULL,					\
			      &mhuv2_data_##n,				\
			      &mhuv2_cfg_##n, POST_KERNEL,		\
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
			      &mhuv2_driver_api);
#endif

DT_INST_FOREACH_STATUS_OKAY(ARM_MHUV2_DATA_INST)
