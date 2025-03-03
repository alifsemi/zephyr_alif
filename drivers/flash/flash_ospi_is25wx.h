/*
 * Copyright (C) 2024 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __FLASH_OSPI_IS25WX_H__
#define __FLASH_OSPI_IS25WX_H__

#include <zephyr/device.h>
#include <ospi_hal.h>

#define OSPI_MAX_RX_COUNT 256

#define OSPI_FLASH_CMD_BUF 261 /* 256 + CMD (1) + ADDRESS (4) */

#define OSPI_FLASH_CMD_READ_STATUS_ERR (0x02)

/* SPI Data Flash Commands */
#define CMD_WRITE_VOL_CONFIG (0x81U)
#define CMD_READ_DATA        (0x7CU)
#define CMD_READ_STATUS      (0x05U)
#define CMD_WRITE_ENABLE     (0x06U)
#define CMD_PAGE_PROGRAM     (0x84U)
#define CMD_READ_FLAG_STATUS (0x70U)
#define CMD_SECTOR_ERASE     (0x21U)
#define CMD_BULK_ERASE       (0xC7U)
#define CMD_READ_DEV_ID      (0x9EU)

#define IO_MODE_ADDRESS    0x00000000U
#define WAIT_CYCLE_ADDRESS 0x00000001U

#define OCTAL_DDR_WO_DQS    (0xC7U)
#define OCTAL_DDR           (0xE7U)
#define DEFAULT_WAIT_CYCLES (0x10U)

/* Flash Driver Status */
#define FLAG_STATUS_BUSY  0x80U
#define FLAG_STATUS_ERROR 0x30U

/* Flash Driver ISSI_Flags */
#define FLASH_INIT  (0x01U)
#define FLASH_POWER (0x02U)

/** SYS_AXI_CLK */
#define SYS_AXI_CLK (400 * 1000 * 1000)

/** Connected chip activate */
#define SLAVE_ACTIVATE    (1)
#define SLAVE_DE_ACTIVATE (0)

/* ISSI XiP WRAP Op-code**/
#define ISSI_XIP_WRAP_CMD	0xFD

/* ISSI XiP INCR Op-code**/
#define ISSI_XIP_INCR_CMD	0xFD


/**IRQ declaration */
typedef void (*irq_config_func_t)(const struct device *dev);

/**Device Configruation */
struct alif_flash_ospi_config {
	irq_config_func_t irq_config;
	uint32_t *regs;                        /* OSPI Reg */
	uint32_t *aes_regs;                    /* AES Reg* */
	struct flash_parameters flash_param;   /* Flash Parameter */
	const struct pinctrl_dev_config *pcfg; /* PINCTRL */
};

/**Device Data */
struct alif_flash_ospi_dev_data {
	HAL_OSPI_Handle_T ospi_handle; /* HAL Handler*/
	uint8_t ISSI_Flags;            /* Flash Init Status*/

	struct k_sem sem;       /* Semaphore */
	struct k_event event_f; /* Event */

	struct ospi_trans_config trans_conf;  /* Transfer Configs */
	uint32_t cmd_buf[OSPI_FLASH_CMD_BUF]; /* CMD + DATA Buffer */
};

#endif /* __FLASH_ALIF_OSPI_H__ */
