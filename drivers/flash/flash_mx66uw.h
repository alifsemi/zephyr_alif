/*
 * Copyright (C) 2026 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __FLASH_OSPI_MX66UW_H__
#define __FLASH_OSPI_MX66UW_H__

#include <zephyr/device.h>
#include <ospi_hal.h>

#define MAX_SEM_TIMEOUT   100

#define OSPI_MAX_RX_COUNT 256

#define OSPI_MAX_TX_COUNT 128

#define OSPI_FLASH_CMD_BUF 261 /* 256 + CMD (1) + ADDRESS (4) */

enum mx66_op_mode {
	MX66_SPI_MODE,
	MX66_OSPI_STR_MODE,
	MX66_OSPI_DTR_MODE
};

/* MX66UW Manufacturer ID*/
#define MX_MANUFACTURER_ID		0xC2

/* *********** MX66UW Commands ******************/

/**Write Enable Latch */
#define	MX_SPI_WREN_CMD			0x06U
#define MX_OSPI_WREN_CMD		0x06F9U

/**Write Volatile ConfigReg2 */
#define MX_SPI_WRCR2_CMD		0x72U
#define MX_OSPI_WRCR2_CMD		0x728DU

/**Erase */
#define MX_OSPI_SEC_ERASE_CMD		0x21DEU
#define MX_OSPI_CHIP_ERASE_CMD		0x609FU

/**Read device Status data */
#define	MX_SPI_READ_STATUS_REG_CMD	0x05U
#define MX_OSPI_READ_STATUS_REG_CMD	0x05FAU

/**Get device ID */
#define MX_SPI_READ_DEV_ID_CMD		0x9FU
#define MX_OSPI_READ_DEV_ID_CMD		0x9F60U

/**Read Security Reg Value */
#define MX_OSPI_READ_SEC_REG_CMD	0x2BD4U

/**Read Contents */
#define MX_OSPI_READ_DATA_CMD		0xEE11U

/**Write Contents */
#define MX_OSPI_PAGE_PRGRM_CMD		0x12EDU


/* MX66UW Config Register Address */
#define	MX_CONF_REG2_OP_MODE_ADDR	0x00000000U
#define MX_CONF_REG2_DQS_EN_ADDR	0x00000200U
#define MX_CONF_REG2_DUMMY_CYL_ADDR	0x00000300U

/* MX66UW OP Mode */
#define MX_OP_MODE_STR_OPI_ENABLE	((0x1) << 0)
#define MX_OP_MODE_DTR_OPI_ENABLE	((0x1) << 1)

/* MX66UW Status values */
#define MX_DEV_STATUS_WIP		((0x1) << 0)
#define MX_DEV_STATUS_WLE		((0x1) << 1)

/** Connected chip activate */
#define MX_CHIP_ACTIVATE		(1)
#define MX_CHIP_DE_ACTIVATE		(0)

/* Flash Driver Status */
#define MX_STATUS_WIP			0x01
#define MX_STATUS_WLE			0x02

/**Error status */
#define MX_SEC_ERASE_ERR_STATUS		BIT(6)
#define MX_SEC_PROGR_ERR_STATUS		BIT(5)

/** Flash dymmy cycles */
#define MX_READ_REG_DUMMY_CYCLES	4
#define MX_ERASE_DYMMY_CYCLES		4
#define MX_READ_DUMMY_CYCLES		0
#define MX_WRITE_DUMMY_CYCLES		0

/** Read Command Dummy Cycles */
#define MX_DATA_READ_DUMMY_CYCLES_AT_200_MHZ	20
#define MX_DATA_READ_DUMMY_CYCLES_AT_173_MHZ	18
#define MX_DATA_READ_DUMMY_CYCLES_AT_166_MHZ	16
#define MX_DATA_READ_DUMMY_CYCLES_AT_155_MHZ	14
#define MX_DATA_READ_DUMMY_CYCLES_AT_133_MHZ	12
#define MX_DATA_READ_DUMMY_CYCLES_AT_104_MHZ	10
#define MX_DATA_READ_DUMMY_CYCLES_AT_084_MHZ	8
#define MX_DATA_READ_DUMMY_CYCLES_AT_066_MHZ	6

/** Default Dummy Cycles */
#define MX_DUMMY_CYCLES_REG_VAL_AT_200_MHZ	0
#define MX_DUMMY_CYCLES_REG_VAL_AT_173_MHZ	1
#define MX_DUMMY_CYCLES_REG_VAL_AT_166_MHZ	2
#define MX_DUMMY_CYCLES_REG_VAL_AT_155_MHZ	3
#define MX_DUMMY_CYCLES_REG_VAL_AT_133_MHZ	4
#define MX_DUMMY_CYCLES_REG_VAL_AT_104_MHZ	5
#define MX_DUMMY_CYCLES_REG_VAL_AT_084_MHZ	6
#define MX_DUMMY_CYCLES_REG_VAL_AT_066_MHZ	7

/** READ STATUS REG timeout */
#define MX_REG_READ_POLL_TIMEOUT_10_MS		10
#define MX_REG_READ_POLL_TIMEOUT_100_MS		100
#define MX_REG_READ_POLL_TIMEOUT_1_MIN		(1*60*1000)
#define MX_REG_READ_POLL_TIMEOUT_2_MIN		(2*60*1000)

/**IRQ declaration */
typedef void (*irq_config_func_t)(const struct device *dev);

/**Device Configruation */
struct mx_flash_ospi_config {
	irq_config_func_t irq_config;
	uint32_t *regs;                        /* OSPI Reg */
	uint32_t *aes_regs;                    /* AES Reg* */
	struct flash_parameters flash_param;   /* Flash Parameter */
	struct flash_pages_layout       flash_layout;
	const struct pinctrl_dev_config *pcfg; /* PINCTRL */
};

/**Device Data */
struct mx_flash_ospi_dev_data {
	HAL_OSPI_Handle_T ospi_handle; /* HAL Handler*/
	struct k_sem sem;              /* Semaphore */
	struct k_event event_f;        /* Event */
	struct ospi_trans_config trans_conf;  /* Transfer Configs */
	uint32_t rw_dfs;                      /* R/W block size*/
	uint32_t cmd_buf[OSPI_FLASH_CMD_BUF]; /* CMD + DATA Buffer */
};

#endif
