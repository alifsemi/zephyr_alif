/*
 * Copyright (C) 2024 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT issi_xspi_flash_controller

#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include "flash_ospi_is25wx.h"

LOG_MODULE_REGISTER(OSPI_FLASH, CONFIG_FLASH_LOG_LEVEL);

#define OSPI_FLASH_NODE DT_NODELABEL(ospi_flash)
#define OSPI_CTRL_NODE  DT_PARENT(OSPI_FLASH_NODE)

#define OSPI_AES_REG_NODE_NAME aes_reg

#define ADDR_IS_SEC_ALIGNED(addr, _bits) ((addr)&BIT_MASK(_bits))
#define FLASH_SEC_SIZE_BIT               12

static void flash_alif_ospi_irq_config_func(const struct device *dev);

static int get_dfs(int block_size)
{
	switch (block_size) {
	case 1:
		return OSPI_DFS_BITS_8;
	case 2:
		return OSPI_DFS_BITS_16;
	case 4:
		return OSPI_DFS_BITS_32;
	default:
		return 0;
	}
}

static inline int32_t err_map_alif_hal_to_zephyr(int32_t err)
{
	int e_code;

	switch (err) {
	case OSPI_ERR_NONE:
		e_code = 0;
		break;
	case OSPI_ERR_INVALID_PARAM:
	case OSPI_ERR_INVALID_HANDLE:
		e_code = -EINVAL;
		break;
	case OSPI_ERR_INVALID_STATE:
		e_code = -EPERM;
		break;
	case OSPI_ERR_CTRL_BUSY:
		e_code = -EBUSY;
		break;
	default:
		e_code = -EIO;
	}

	return e_code;
}

static void hal_event_update(uint32_t event_status, void *user_data)
{
	struct alif_flash_ospi_dev_data *dev_data = (struct alif_flash_ospi_dev_data *)user_data;

	k_event_post(&dev_data->event_f, event_status);
}

static inline int32_t set_cs_pin(HAL_OSPI_Handle_T handle, int activate)
{
	int ret;

	do {
		ret = alif_hal_ospi_cs_enable(handle, activate);
	} while (ret == OSPI_ERR_CTRL_BUSY);

	ret = err_map_alif_hal_to_zephyr(ret);
	return ret;
}

static int32_t read_status_reg(const struct device *dev, uint8_t command, uint8_t *stat)
{
	int32_t ret;
	uint32_t event, cmd_buf[2] = {0};

	struct alif_flash_ospi_dev_data *dev_data = dev->data;

	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_0_BITS;
	dev_data->trans_conf.ddr_enable = OSPI_DDR_ENABLE;
	dev_data->trans_conf.wait_cycles = 8;

	cmd_buf[0] = command;
	*stat = OSPI_FLASH_CMD_READ_STATUS_ERR;

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	k_event_clear(&dev_data->event_f, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	ret = alif_hal_ospi_transfer(dev_data->ospi_handle, &cmd_buf[0], &cmd_buf[1], 1);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	event = k_event_wait(&dev_data->event_f,
			     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false, K_FOREVER);

	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		/* De-Select Slave */
		set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
		return -EIO;
	}

	/* De-Select Slave */
	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	*stat = (uint8_t)cmd_buf[1];
	return ret;
}

static int set_write_enable(const struct device *dev, int dfs)
{
	uint32_t cmd_buf, event;
	uint8_t val = 0;

	struct alif_flash_ospi_dev_data *dev_data = dev->data;
	int ret;

	/*Prepare for Tx*/
	dev_data->trans_conf.frame_size = dfs;

	/*Prepare Interface for Initialize Flash*/
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	k_event_clear(&dev_data->event_f, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	/*Select Slave*/
	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	cmd_buf = CMD_WRITE_ENABLE;

	LOG_DBG("Enable Flash for Write ");

	ret = alif_hal_ospi_send(dev_data->ospi_handle, &cmd_buf, 1U);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	event = k_event_wait(&dev_data->event_f,
			     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false, K_FOREVER);
	/* Check the Event Status*/
	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
		return -EIO;
	}

	/*De-Select*/
	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	/* Read Flag status works after switched to Octal-DDR */
	if (dev_data->ISSI_Flags & FLASH_POWER) {
		LOG_DBG("read flash status reg");
		ret = read_status_reg(dev, CMD_READ_STATUS, &val);
		if ((ret == 0) && ((val & OSPI_FLASH_CMD_READ_STATUS_ERR) == 0)) {
			return -EIO;
		}
	}
	return ret;
}

static int flash_is25wx_ospi_read(const struct device *dev, off_t address, void *buffer,
				size_t length)
{
	uint32_t cmd[4], data_cnt, event;
	uint8_t *data_ptr;
	int32_t ret;

	const struct alif_flash_ospi_config *dev_cfg = dev->config;
	const struct flash_parameters *f_param = &dev_cfg->flash_param;
	struct alif_flash_ospi_dev_data *dev_data = dev->data;

	/*Verify Address boundary*/
	if ((address > (f_param->num_of_sector * f_param->sector_size)) || (address < 0) ||
	    (buffer == NULL) ||
	    ((address + length) > (f_param->num_of_sector * f_param->sector_size))) {
		return -EINVAL;
	}

	/*In DDR Mode bytes and length should be in multiple of write_block_size */
	if (length % f_param->write_block_size
		|| !(IS_ALIGNED(buffer, f_param->write_block_size))) {
		return -EINVAL;
	}

	/* Lock */
	ret = k_sem_take(&dev_data->sem, K_MSEC(MAX_SEM_TIMEOUT));
	if (ret != 0) {
		return ret;
	}

	LOG_DBG("read address %u length to read %d", (uint32_t) address, length);

	/* number of block count */
	length /= f_param->write_block_size;

	data_ptr = (uint8_t *) buffer;

	dev_data->trans_conf.wait_cycles = 16;
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
	dev_data->trans_conf.frame_size = dev_cfg->rw_dfs;

	/* Prepare Interface and update Configuration */
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		k_sem_give(&dev_data->sem);
		return ret;
	}

	while (length) {
		ret = set_cs_pin(dev_data->ospi_handle, SLAVE_ACTIVATE);
		if (ret != 0) {
			break;
		}

		data_cnt = OSPI_MAX_RX_COUNT;
		if (data_cnt > length) {
			data_cnt = length;
		}

		/* Prepare command with address */
		cmd[0] = CMD_READ_DATA;
		cmd[1] = address;

		k_event_clear(&dev_data->event_f,
			      OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

		ret = alif_hal_ospi_transfer(dev_data->ospi_handle, cmd, data_ptr, data_cnt);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		event = k_event_wait(&dev_data->event_f,
				     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false,
				     K_FOREVER);

		if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
			LOG_ERR("F-Read: Incomplete Event [%d]", event);
			set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
			ret = -EIO;
			break;
		}

		ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
		if (ret != 0) {
			break;
		}

		length -= data_cnt;

		/* Update address and data offset with configured block size*/
		address += (data_cnt * f_param->write_block_size);
		data_ptr += (data_cnt * f_param->write_block_size);
	}

	k_sem_give(&dev_data->sem);
	return ret;
}

static int flash_is25wx_ospi_write(const struct device *dev, off_t address, const void *buffer,
				 size_t length)
{
	const uint8_t *data_ptr;
	int32_t status, ret;
	uint32_t event, data_cnt, index, i, cnt, data_i = 0;
	uint8_t val;

	const struct alif_flash_ospi_config *dev_cfg = dev->config;
	const struct flash_parameters *f_param = &dev_cfg->flash_param;
	struct alif_flash_ospi_dev_data *dev_data = dev->data;

	LOG_DBG("write address %u length to write %d", (uint32_t) address, length);

	/*Verify Address boundary*/
	if ((address > (f_param->num_of_sector * f_param->sector_size)) || (buffer == NULL) ||
	    (address < 0) ||
	    ((address + length) > (f_param->num_of_sector * f_param->sector_size))) {
		return -EINVAL;
	}

	/*In DDR Mode bytes and length should be in multiple of write_block_size */
	if (length % f_param->write_block_size
		|| !(IS_ALIGNED(buffer, f_param->write_block_size))) {
		return -EINVAL;
	}

	ret = k_sem_take(&dev_data->sem, K_MSEC(MAX_SEM_TIMEOUT));
	if (ret != 0) {
		return ret;
	}

	/* number of block count */
	cnt = length / f_param->write_block_size;
	data_ptr = (uint8_t *) buffer;

	while (cnt) {
		dev_data->trans_conf.wait_cycles = 0;
		dev_data->trans_conf.addr_len = 0;

		ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		ret = set_write_enable(dev, OSPI_DFS_BITS_16);
		if (ret != 0) {
			break;
		}

		data_cnt = (OSPI_MAX_TX_COUNT - (address % OSPI_MAX_TX_COUNT));
		if (data_cnt > cnt) {
			data_cnt = cnt;
		}

		/* Prepare command with address */
		dev_data->cmd_buf[0] = CMD_PAGE_PROGRAM;
		dev_data->cmd_buf[1] = address;

		index = 2;

		for (i = 0; i < data_cnt; i++) {
			switch (dev_cfg->rw_dfs) {
			case OSPI_DFS_BITS_8:
				dev_data->cmd_buf[index++] = data_ptr[data_i];
				break;
			case OSPI_DFS_BITS_16:
				dev_data->cmd_buf[index++] =
					*(uint16_t *) (data_ptr + (data_i * sizeof(uint16_t)));
				break;
			case OSPI_DFS_BITS_32:
				dev_data->cmd_buf[index++] =
					*(uint32_t *) (data_ptr + (data_i * sizeof(uint32_t)));
				break;
			}
			data_i++;
		}

		dev_data->trans_conf.wait_cycles = 0;
		dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
		dev_data->trans_conf.frame_size = dev_cfg->rw_dfs;

		ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		ret = set_cs_pin(dev_data->ospi_handle, SLAVE_ACTIVATE);
		if (ret != 0) {
			break;
		}

		k_event_clear(&dev_data->event_f,
			      OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

		ret = alif_hal_ospi_send(dev_data->ospi_handle, dev_data->cmd_buf, data_cnt + 2);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		event = k_event_wait(&dev_data->event_f,
				     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false,
				     K_FOREVER);

		if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
			LOG_ERR("F-Write: Incomplete Event [%d]", event);
			/* De-Select slave */
			set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
			ret = -EIO;
			break;
		}

		/* Update address offset with configured block size */
		address += (data_cnt * f_param->write_block_size);
		cnt -= data_cnt;

		ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
		if (ret != 0) {
			break;
		}

		/* Read status until device ready */
		do {
			status = read_status_reg(dev, CMD_READ_FLAG_STATUS, &val);
			if (status != 0) {
				break;
			}

			if ((val & FLAG_STATUS_ERROR) != 0U) {

				LOG_ERR("F-Write: Read Flash status [%d]", val);
				ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
				if (ret != 0) {
					break;
				}
				ret = -EIO;
				break;
			}
		} while ((val & FLAG_STATUS_BUSY) == 0);

		if (ret != 0) {
			break;
		}
	}

	/* Force De-Select */
	set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);

	k_sem_give(&dev_data->sem);
	return ret;
}

static int erase_chip(const struct device *dev)
{
	int ret;
	struct alif_flash_ospi_dev_data *dev_data = dev->data;
	uint32_t cmd[2];

	dev_data->trans_conf.wait_cycles = 0;
	dev_data->trans_conf.addr_len = 0;

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	cmd[0] = CMD_BULK_ERASE;

	ret = alif_hal_ospi_send(dev_data->ospi_handle, cmd, 1);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int erase_sector(const struct device *dev, uint32_t addr)
{
	int ret;
	struct alif_flash_ospi_dev_data *dev_data = dev->data;
	uint32_t cmd[2];

	dev_data->trans_conf.wait_cycles = 0;
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	cmd[0] = CMD_SECTOR_ERASE;
	cmd[1] = addr;

	ret = alif_hal_ospi_send(dev_data->ospi_handle, cmd, ARRAY_SIZE(cmd));
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int flash_is25wx_ospi_erase(const struct device *dev, off_t addr, size_t len)
{
	int ret;
	uint8_t val;
	uint32_t event;

	struct alif_flash_ospi_dev_data *dev_data = dev->data;
	const struct alif_flash_ospi_config *dev_config = dev->config;
	const struct flash_parameters *f_param = &dev_config->flash_param;

	/* address range should be within the flash size*/
	if ((addr < 0 || addr > (f_param->num_of_sector * f_param->sector_size)) ||
	    ((addr + len) > (f_param->num_of_sector * f_param->sector_size))) {
		return -EINVAL;
	}

	/* Length should be a multiple of sector size and valid */
	if ((len % f_param->sector_size) != 0 || len == 0) {
		return -EINVAL;
	}

	/* Address should be aligned */
	if (ADDR_IS_SEC_ALIGNED(addr, FLASH_SEC_SIZE_BIT)) {
		return -EINVAL;
	}

	ret = k_sem_take(&dev_data->sem, K_NO_WAIT);
	if (ret != 0) {
		return ret;
	}

	LOG_DBG("write address %u length for erase %d", (uint32_t) addr, len);

	while (len) {
		dev_data->trans_conf.wait_cycles = 0;
		dev_data->trans_conf.addr_len = 0;

		ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		ret = set_write_enable(dev, OSPI_DFS_BITS_16);
		if (ret != 0) {
			break;
		}

		k_event_clear(&dev_data->event_f,
			      OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

		if (len == (dev_config->flash_param.sector_size *
				dev_config->flash_param.num_of_sector)) {
			ret = erase_chip(dev);
			if (ret != 0) {
				break;
			}
			len -= dev_config->flash_param.sector_size *
			       dev_config->flash_param.num_of_sector;
		} else {
			ret = erase_sector(dev, addr);
			if (ret != 0) {
				break;
			}
			len -= dev_config->flash_param.sector_size;
			addr += dev_config->flash_param.sector_size;
		}

		event = k_event_wait(&dev_data->event_f,
				     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false,
				     K_FOREVER);

		if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
			LOG_ERR("F-Erase: Incomplete Event [%d]", event);
			set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
			ret = -EIO;
			break;
		}

		do {
			ret = read_status_reg(dev, CMD_READ_FLAG_STATUS, &val);
			if (ret != 0) {
				break;
			}

			/* Check Program and Erase bits */
			if ((val & FLAG_STATUS_ERROR) != 0U) {
				LOG_ERR("F-Erase: Read Flash status [%d]", val);
				ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
				if (ret != 0) {
					break;
				}
				ret = -EIO;
				break;
			}
		} while ((val & FLAG_STATUS_BUSY) == 0);

		if (ret != 0) {
			break;
		}
	}

	/* Force De-Select */
	set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);

	k_sem_give(&dev_data->sem);

	return ret;
}

static int flash_is25wx_ospi_init(const struct device *dev)
{
	int ret, dfs;
	uint32_t event;

	struct alif_flash_ospi_config *dev_cfg = (struct alif_flash_ospi_config *)dev->config;
	struct alif_flash_ospi_dev_data *dev_data = (struct alif_flash_ospi_dev_data *)dev->data;

	struct ospi_init init_config;

	memset(&init_config, 0, sizeof(struct ospi_init));

	init_config.core_clk = DT_PROP(OSPI_CTRL_NODE, clock_frequency);
	init_config.bus_speed = DT_PROP(OSPI_CTRL_NODE, bus_speed);
	init_config.tx_fifo_threshold = DT_PROP(OSPI_CTRL_NODE, tx_fifo_threshold);
	init_config.rx_fifo_threshold = 0;
	init_config.rx_sample_delay = 0;
	init_config.ddr_drive_edge = DT_PROP(OSPI_CTRL_NODE, ddr_drive_edge);
	init_config.cs_pin = DT_PROP(OSPI_CTRL_NODE, cs_pin);
	init_config.rx_ds_delay = DT_PROP(OSPI_CTRL_NODE, rx_ds_delay);
	init_config.baud2_delay = DT_PROP(OSPI_CTRL_NODE, baud2_delay);
	init_config.base_regs = dev_cfg->regs;
	init_config.aes_regs = dev_cfg->aes_regs;
	init_config.event_cb = hal_event_update;
	init_config.user_data = dev_data;

	init_config.xip_incr_cmd = ISSI_XIP_INCR_CMD;
	init_config.xip_wrap_cmd = ISSI_XIP_WRAP_CMD;
	init_config.xip_rxds_vl_en = DT_PROP(OSPI_CTRL_NODE, xip_rxds_vl_en);
	init_config.xip_wait_cycles = DT_PROP(OSPI_CTRL_NODE, xip_wait_cycles);

	memset(&dev_data->trans_conf, 0, sizeof(struct ospi_trans_config));

	dev_data->trans_conf.frame_size = OSPI_DFS_BITS_8;
	dev_data->trans_conf.frame_format = OSPI_FRF_STANDRAD;
	dev_data->trans_conf.addr_len = 0;
	dev_data->trans_conf.wait_cycles = 0;
	dev_data->trans_conf.ddr_enable = OSPI_DDR_DISABLE;
	dev_data->trans_conf.inst_len = OSPI_INST_LENGTH_8_BITS;
	dev_data->trans_conf.trans_type = 0;
	dev_data->trans_conf.rx_ds_enable = 0;

	/* initialize semaphore */
	k_sem_init(&dev_data->sem, 1, 1);
	k_event_init(&dev_data->event_f);

	pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);

	/* IRQ Init */
	dev_cfg->irq_config(dev);

	ret = alif_hal_ospi_initialize(&dev_data->ospi_handle, &init_config);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}
	/* Initialize Configuration */
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_write_enable(dev, OSPI_DFS_BITS_8);
	if (ret != 0) {
		return ret;
	}

	/* Prepare command and address for setting flash in octal mode */
	if (IS_ENABLED(CONFIG_FLASH_ADDRESS_IN_SINGLE_FIFO_LOCATION)) {
		dev_data->cmd_buf[0] = CMD_WRITE_VOL_CONFIG;
		dev_data->cmd_buf[1] = (uint8_t)(IO_MODE_ADDRESS >> 0);
		dev_data->cmd_buf[2] = OCTAL_DDR;
	} else {
		dev_data->cmd_buf[0] = CMD_WRITE_VOL_CONFIG;
		dev_data->cmd_buf[1] = (uint8_t)(IO_MODE_ADDRESS >> 16);
		dev_data->cmd_buf[2] = (uint8_t)(IO_MODE_ADDRESS >> 8);
		dev_data->cmd_buf[3] = (uint8_t)(IO_MODE_ADDRESS >> 0);
		dev_data->cmd_buf[4] = OCTAL_DDR;
	}
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_24_BITS;

	/* Select Chip */
	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	k_event_clear(&dev_data->event_f, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	ret = alif_hal_ospi_send(dev_data->ospi_handle,
			dev_data->cmd_buf, CONFIG_FLASH_PREPARE_CMD_LEN);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	event = k_event_wait(&dev_data->event_f,
			     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false, K_FOREVER);

	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		/* De-Select slave */
		set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
		return -EIO;
	}

	/* De-Select slave */
	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	/* Set Default Wait Cycle */
	dev_data->cmd_buf[0] = CMD_WRITE_VOL_CONFIG;
	dev_data->cmd_buf[1] = WAIT_CYCLE_ADDRESS;
	dev_data->cmd_buf[2] = DEFAULT_WAIT_CYCLES;
	dev_data->cmd_buf[3] = DEFAULT_WAIT_CYCLES;

	/** Update DFS */
	dfs = get_dfs(dev_cfg->flash_param.write_block_size);
	if (dfs == 0) {
		return -EINVAL;
	}

	/** R-W FrameSize */
	dev_cfg->rw_dfs = dfs;

	dev_data->trans_conf.frame_size = OSPI_DFS_BITS_16;
	dev_data->trans_conf.frame_format = OSPI_FRF_OCTAL;
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_0_BITS;
	dev_data->trans_conf.wait_cycles = 0;
	dev_data->trans_conf.ddr_enable = OSPI_DDR_ENABLE;

	/* Update configuration */
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_write_enable(dev, OSPI_DFS_BITS_16);
	if (ret != 0) {
		return ret;
	}

	/* Re-configure */
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
	dev_data->trans_conf.ddr_enable = OSPI_DDR_ENABLE;

	/* Update Interface configuration */
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	/* Select Chip */
	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	k_event_clear(&dev_data->event_f, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	/* Send Command */
	ret = alif_hal_ospi_send(dev_data->ospi_handle, dev_data->cmd_buf, 4);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	event = k_event_wait(&dev_data->event_f,
			     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false, K_FOREVER);

	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		/* De-Select slave */
		set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
		return -EIO;
	}

	/* De-Select slave */
	ret = set_cs_pin(dev_data->ospi_handle, SLAVE_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	dev_data->ISSI_Flags |= FLASH_POWER;

	if (IS_ENABLED(CONFIG_ALIF_OSPI_FLASH_XIP)) {
		alif_hal_ospi_xip_enable(dev_data->ospi_handle);
	}

	return ret;
}


#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_alif_ospi_page_layout(const struct device *dev,
				const struct flash_pages_layout **layout,
				size_t *layout_size)
{
	struct alif_flash_ospi_config *dev_cfg = (struct alif_flash_ospi_config *) dev->config;

	*layout = &(dev_cfg->flash_layout);
	*layout_size = 1;
}
#endif


/* PINCTRL Definition Macro for Node */
PINCTRL_DT_DEFINE(OSPI_CTRL_NODE);

static const struct flash_parameters *flash_is25wx_ospi_get_parameters(const struct device *dev)
{
	struct alif_flash_ospi_config *dev_cfg = (struct alif_flash_ospi_config *)dev->config;

	return &(dev_cfg->flash_param);
}

static const struct flash_driver_api flash_alif_ospi_driver_api = {
	.read = flash_is25wx_ospi_read,
	.write = flash_is25wx_ospi_write,
	.erase = flash_is25wx_ospi_erase,
	.get_parameters = flash_is25wx_ospi_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_alif_ospi_page_layout,
#endif

};

struct alif_flash_ospi_config alif_flash_ospi_config = {
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(OSPI_CTRL_NODE),

	.flash_param.write_block_size = DT_PROP(OSPI_FLASH_NODE, write_block_size),
	.flash_param.erase_value = DT_PROP(OSPI_FLASH_NODE, erase_value),
	.flash_param.num_of_sector = DT_PROP(OSPI_FLASH_NODE, num_of_sector),
	.flash_param.sector_size = DT_PROP(OSPI_FLASH_NODE, sector_size),
	.flash_param.page_size = DT_PROP(OSPI_FLASH_NODE, page_size),

#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.flash_layout.pages_size = DT_PROP(OSPI_FLASH_NODE, page_size),
	.flash_layout.pages_count = DT_PROP(OSPI_FLASH_NODE, num_of_sector) *
		DT_PROP(OSPI_FLASH_NODE, sector_size) / DT_PROP(OSPI_FLASH_NODE, page_size),
#endif

	.regs = (uint32_t *)DT_REG_ADDR(OSPI_CTRL_NODE),
	.aes_regs = (uint32_t *)DT_PROP_BY_IDX(OSPI_CTRL_NODE, OSPI_AES_REG_NODE_NAME, 0),

	.irq_config = flash_alif_ospi_irq_config_func,
};

static struct alif_flash_ospi_dev_data flash_ospi_data = {
	.ISSI_Flags = 0,
};

static void flash_is25wx_ospi_isr(const struct device *dev)
{
	struct alif_flash_ospi_dev_data *dev_data = (struct alif_flash_ospi_dev_data *)dev->data;

	/* ospi-irq handler */
	alif_hal_ospi_irq_handler(dev_data->ospi_handle);
}

DEVICE_DT_DEFINE(OSPI_FLASH_NODE, &flash_is25wx_ospi_init, NULL, &flash_ospi_data,
		 &alif_flash_ospi_config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 &flash_alif_ospi_driver_api);

static void flash_alif_ospi_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(OSPI_CTRL_NODE), DT_IRQ(OSPI_CTRL_NODE, priority),
			flash_is25wx_ospi_isr, DEVICE_DT_GET(OSPI_FLASH_NODE), 0);
	irq_enable(DT_IRQN(OSPI_CTRL_NODE));
}
