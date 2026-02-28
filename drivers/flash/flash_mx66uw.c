/*
 * Copyright (C) 2026 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mxicy_mx66uw_flash

#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include "flash_mx66uw.h"

LOG_MODULE_REGISTER(OSPI_FLASH, CONFIG_FLASH_LOG_LEVEL);

#define OSPI_FLASH_NODE		DT_NODELABEL(ospi_flash)
#define OSPI_CTRL_NODE		DT_PARENT(OSPI_FLASH_NODE)

#define OSPI_AES_REG_NODE_NAME	aes_reg

#define ADDR_IS_SEC_ALIGNED(addr, _bits)	((addr)&BIT_MASK(_bits))
#define FLASH_SEC_SIZE_BIT			12

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
	struct mx_flash_ospi_dev_data *dev_data = (struct mx_flash_ospi_dev_data *)user_data;

	k_event_post(&dev_data->event_f, event_status);
}

static inline int32_t set_cs_pin(HAL_OSPI_Handle_T handle, int state)
{
	int ret;

	do {
		ret = alif_hal_ospi_cs_enable(handle, state);
	} while (ret == OSPI_ERR_CTRL_BUSY);

	ret = err_map_alif_hal_to_zephyr(ret);
	return ret;
}

static int activate_dev(struct mx_flash_ospi_dev_data *dev_data)
{
	int ret;

	/*Clear Event*/
	k_event_clear(&dev_data->event_f, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	/*Activate*/
	ret = set_cs_pin(dev_data->ospi_handle, MX_CHIP_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static int wait_to_complete(struct mx_flash_ospi_dev_data *dev_data)
{
	uint32_t event;
	int32_t ret;

	/*Wait for Event*/
	event = k_event_wait(&dev_data->event_f,
			     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false, K_FOREVER);

	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		/* De-Select Slave */
		set_cs_pin(dev_data->ospi_handle, MX_CHIP_DE_ACTIVATE);
		return -EIO;
	}

	/*De-Select Slave*/
	ret = set_cs_pin(dev_data->ospi_handle, MX_CHIP_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static int read_sec_reg(struct mx_flash_ospi_dev_data *dev_data, uint8_t *sec_val)
{
	uint32_t cmd[2], val;
	int ret;

	dev_data->trans_conf.wait_cycles = MX_READ_REG_DUMMY_CYCLES;
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;

	cmd[0] = MX_OSPI_READ_SEC_REG_CMD;
	cmd[1] = 0x0;

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = activate_dev(dev_data);
	if (ret != 0) {
		return ret;
	}

	ret = alif_hal_ospi_transfer(dev_data->ospi_handle, cmd, &val, 1);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = wait_to_complete(dev_data);
	if (ret != 0) {
		return ret;
	}

	/*Get 8-bits*/
	*sec_val = (uint8_t) val;

	return ret;
}


static int read_dev_id(struct mx_flash_ospi_dev_data *dev_data, uint8_t *val)
{
	uint32_t cmd[2], dev_id;
	int ret;

	dev_data->trans_conf.wait_cycles = MX_READ_REG_DUMMY_CYCLES;
	dev_data->trans_conf.frame_format = SPI_FRF_OCTAL;
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;

	cmd[0] = MX_OSPI_READ_DEV_ID_CMD;
	cmd[1] = 0x0;

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = activate_dev(dev_data);
	if (ret != 0) {
		return ret;
	}

	ret = alif_hal_ospi_transfer(dev_data->ospi_handle, cmd, &dev_id, 1);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = wait_to_complete(dev_data);
	if (ret != 0) {
		return ret;
	}

	/**get 8-bit */
	*val = (dev_id & 0xff);

	return ret;
}

static int read_status_reg(struct mx_flash_ospi_dev_data *dev_data, uint8_t *res_status)
{
	uint32_t status;
	int ret;

	uint32_t cmd_buff[4];

	/**Prepare command and config : READ Status REG*/
	cmd_buff[0] = MX_OSPI_READ_STATUS_REG_CMD;
	cmd_buff[1] = 0x0;

	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
	dev_data->trans_conf.wait_cycles = MX_READ_REG_DUMMY_CYCLES;

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = activate_dev(dev_data);
	if (ret != 0) {
		return ret;
	}

	ret = alif_hal_ospi_transfer(dev_data->ospi_handle, cmd_buff, &status, 1);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = wait_to_complete(dev_data);
	if (ret != 0) {
		return ret;
	}

	/* Update status */
	*res_status = (uint8_t) status;

	return ret;
}

static int poll_read_status_reg(struct mx_flash_ospi_dev_data *dev_data,
				 int timeout_ms)
{
	int ret;
	uint8_t val;
	int64_t t_out = k_uptime_get() + (timeout_ms);

	do {
		val = 0;
		ret = read_status_reg(dev_data, &val);
		if (ret) {
			return -EIO;   /* error */
		}

		if (!(val & MX_STATUS_WIP)) {
			break;        /* WIP cleared */
		}

		if (k_uptime_get() > t_out) {
			return -ETIMEDOUT;
		}
		k_busy_wait(10);  /* 10us poll delay */
	} while (1);

	return 0;
}

static int set_dtr_ospi_mode(struct mx_flash_ospi_dev_data *dev_data)
{
	uint32_t cmd_buff[4];
	int ret;

	/**Prepare command and config */
	dev_data->trans_conf.frame_size = OSPI_DFS_BITS_8;
	dev_data->trans_conf.inst_len  = OSPI_INST_LENGTH_8_BITS;
	dev_data->trans_conf.frame_format = OSPI_FRF_STANDRAD;
	dev_data->trans_conf.ddr_enable = OSPI_DDR_DISABLE;
	dev_data->trans_conf.wait_cycles = 0;
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;

	cmd_buff[0] = MX_SPI_WRCR2_CMD;
	cmd_buff[1] = MX_CONF_REG2_OP_MODE_ADDR;
	cmd_buff[2] = MX_OP_MODE_DTR_OPI_ENABLE;

	/*Prepare for Tx*/
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = activate_dev(dev_data);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	LOG_DBG("Req: turn to DTR-OSPI Mode");

	ret = alif_hal_ospi_send(dev_data->ospi_handle, cmd_buff, 3);
	if (ret != 0) {
		return ret;
	}

	ret = wait_to_complete(dev_data);
	if (ret != 0) {
		return ret;
	}

	return 0;
}


static int update_dummy_cycle(struct mx_flash_ospi_dev_data *dev_data, int dummy_cyl)
{
	uint32_t cmd_buff[4];
	int ret;

	/**Prepare command and config */
	cmd_buff[0] = MX_OSPI_WRCR2_CMD;
	cmd_buff[1] = MX_CONF_REG2_DUMMY_CYL_ADDR;
	cmd_buff[2] = ((dummy_cyl << 8) | 0x0);   /*shift value byte :16-bit*/

	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
	dev_data->trans_conf.wait_cycles = 0;

	/*Prepare for Tx*/
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = activate_dev(dev_data);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	LOG_DBG("Req: Update dummy cycle");

	ret = alif_hal_ospi_send(dev_data->ospi_handle, cmd_buff, 3);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = wait_to_complete(dev_data);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static int set_write_enable(struct mx_flash_ospi_dev_data *dev_data, enum mx66_op_mode op_mode)
{
	uint32_t cmd;
	int ret;

	/*Write latch enable */
	cmd = (op_mode == MX66_SPI_MODE) ? MX_SPI_WREN_CMD : MX_OSPI_WREN_CMD;

	switch (op_mode) {
	case MX66_SPI_MODE:
		dev_data->trans_conf.frame_size = OSPI_DFS_BITS_8;
		dev_data->trans_conf.inst_len  = OSPI_INST_LENGTH_8_BITS;
		dev_data->trans_conf.frame_format = OSPI_FRF_STANDRAD;
		dev_data->trans_conf.ddr_enable = OSPI_DDR_DISABLE;
		break;

	case MX66_OSPI_STR_MODE:
		dev_data->trans_conf.frame_size = OSPI_DFS_BITS_8;
		dev_data->trans_conf.inst_len  = OSPI_INST_LENGTH_16_BITS;
		dev_data->trans_conf.frame_format = OSPI_FRF_OCTAL;
		dev_data->trans_conf.ddr_enable = OSPI_DDR_DISABLE;
		break;

	case MX66_OSPI_DTR_MODE:
		dev_data->trans_conf.frame_size = OSPI_DFS_BITS_16;
		dev_data->trans_conf.inst_len  = OSPI_INST_LENGTH_16_BITS;
		dev_data->trans_conf.frame_format = OSPI_FRF_OCTAL;
		dev_data->trans_conf.ddr_enable = OSPI_DDR_ENABLE;
		break;

	default:
		break;
	};

	/*Wait cycle */
	dev_data->trans_conf.wait_cycles = 0;

	/*Address len */
	dev_data->trans_conf.addr_len = 0;

	ret = activate_dev(dev_data);
	if (ret != 0) {
		return ret;
	}

	/*Prepare for Tx*/
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	LOG_DBG("Req: Write Latch Enable");

	ret = alif_hal_ospi_send(dev_data->ospi_handle, &cmd, 1U);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = wait_to_complete(dev_data);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static int flash_mx66uw_read(const struct device *dev, off_t address, void *buffer,
				size_t length)
{
	uint32_t cmd[4], data_cnt;
	uint8_t *data_ptr;
	int32_t ret;

	const struct mx_flash_ospi_config *dev_cfg = dev->config;
	const struct flash_parameters *f_param = &dev_cfg->flash_param;
	struct mx_flash_ospi_dev_data *dev_data = dev->data;

	/*Verify Address boundary*/
	if ((address > (f_param->num_of_sector * f_param->sector_size)) || (address < 0) ||
	    (buffer == NULL) ||
	    ((address + length) > (f_param->num_of_sector * f_param->sector_size))) {
		return -EINVAL;
	}

	/*In DTR-OSPI Mode bytes and length should be in multiple of write_block_size */
	if (length % f_param->write_block_size
		|| !(IS_ALIGNED(buffer, f_param->write_block_size))) {
		return -EINVAL;
	}

	/* Lock */
	ret = k_sem_take(&dev_data->sem, K_MSEC(MAX_SEM_TIMEOUT));
	if (ret != 0) {
		return ret;
	}

	length /= f_param->write_block_size;
	data_ptr = (uint8_t *)buffer;

	LOG_DBG("read address %u length to read %d", (uint32_t) address, length);

	dev_data->trans_conf.wait_cycles = DT_PROP(OSPI_FLASH_NODE, read_wait_cycle);
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
	dev_data->trans_conf.frame_size = dev_data->rw_dfs;

	/* Prepare Interface and update Configuration */
	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		k_sem_give(&dev_data->sem);
		return ret;
	}

	while (length) {
		data_cnt = OSPI_MAX_RX_COUNT;
		if (data_cnt > length) {
			data_cnt = length;
		}

		/* Prepare command with address */
		cmd[0] = MX_OSPI_READ_DATA_CMD;
		cmd[1] = address;

		ret = activate_dev(dev_data);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		ret = alif_hal_ospi_transfer(dev_data->ospi_handle, cmd, data_ptr, data_cnt);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		ret = wait_to_complete(dev_data);
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

static int flash_mx66uw_write(const struct device *dev, off_t address, const void *buffer,
				 size_t length)
{
	uint8_t *data_ptr, val;
	int32_t ret, data_cnt, index, i, cnt, data_i = 0;

	const struct mx_flash_ospi_config *dev_cfg = dev->config;
	const struct flash_parameters *f_param = &dev_cfg->flash_param;
	struct mx_flash_ospi_dev_data *dev_data = dev->data;

	LOG_DBG("write address %u length to write %d", (uint32_t) address, length);

	/*Verify Address boundary*/
	if ((address > (f_param->num_of_sector * f_param->sector_size)) || (buffer == NULL) ||
	    (address < 0) ||
	    ((address + length) > (f_param->num_of_sector * f_param->sector_size))) {
		return -EINVAL;
	}

	/*In DTR-OPI Mode bytes and length should be in multiple of write_block_size */
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
	data_ptr = (uint8_t *)buffer;

	while (cnt) {
		/* Address Length for WLE command */
		dev_data->trans_conf.addr_len = 0;

		ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		ret = set_write_enable(dev_data, MX66_OSPI_DTR_MODE);
		if (ret != 0) {
			break;
		}

		data_cnt = (OSPI_MAX_TX_COUNT - (address % OSPI_MAX_TX_COUNT));
		if (data_cnt > cnt) {
			data_cnt = cnt;
		}

		/* Prepare command with address */
		dev_data->cmd_buf[0] = MX_OSPI_PAGE_PRGRM_CMD;
		dev_data->cmd_buf[1] = address;

		index = 2;

		for (i = 0; i < data_cnt; i++) {
			switch (dev_data->rw_dfs) {
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

		/* Address Length for Write command */
		dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
		dev_data->trans_conf.frame_size = dev_data->rw_dfs;

		ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		ret = activate_dev(dev_data);
		if (ret != 0) {
			break;
		}

		ret = alif_hal_ospi_send(dev_data->ospi_handle, dev_data->cmd_buf, data_cnt + 2);
		if (ret != 0) {
			ret = err_map_alif_hal_to_zephyr(ret);
			break;
		}

		ret = wait_to_complete(dev_data);
		if (ret != 0) {
			break;
		}

		address += (data_cnt * f_param->write_block_size);
		cnt -= data_cnt;

		/** Read status with timeout */
		ret = poll_read_status_reg(dev_data, MX_REG_READ_POLL_TIMEOUT_100_MS);
		if (ret != 0) {
			break;
		}

		/**Check Program status bit */
		val = 0xFF;
		ret = read_sec_reg(dev_data, &val);
		if ((val & MX_SEC_PROGR_ERR_STATUS) || ret != 0) {
			ret = -EIO;
			break;
		}
	}

	/* Force De-Select */
	set_cs_pin(dev_data->ospi_handle, MX_CHIP_DE_ACTIVATE);

	k_sem_give(&dev_data->sem);
	return ret;
}

static int erase_chip(const struct device *dev)
{
	int ret;
	struct mx_flash_ospi_dev_data *dev_data = dev->data;
	uint32_t cmd[2];

	dev_data->trans_conf.wait_cycles = 0;
	dev_data->trans_conf.addr_len = 0;

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, MX_CHIP_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	cmd[0] = MX_OSPI_CHIP_ERASE_CMD;

	ret = alif_hal_ospi_send(dev_data->ospi_handle, cmd, 1);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, MX_CHIP_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int erase_sector(const struct device *dev, uint32_t addr)
{
	int ret;
	struct mx_flash_ospi_dev_data *dev_data = dev->data;
	uint32_t cmd[2];

	dev_data->trans_conf.wait_cycles = 0;
	dev_data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;

	ret = alif_hal_ospi_prepare_transfer(dev_data->ospi_handle, &dev_data->trans_conf);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, MX_CHIP_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	cmd[0] = MX_OSPI_SEC_ERASE_CMD;
	cmd[1] = addr;

	ret = alif_hal_ospi_send(dev_data->ospi_handle, cmd, ARRAY_SIZE(cmd));
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		return ret;
	}

	ret = set_cs_pin(dev_data->ospi_handle, MX_CHIP_DE_ACTIVATE);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int flash_mx66uw_erase(const struct device *dev, off_t addr, size_t len)
{
	int ret, poll_timeout;
	uint8_t val = 0xff;
	uint32_t event;

	struct mx_flash_ospi_dev_data *dev_data = dev->data;
	const struct mx_flash_ospi_config *dev_config = dev->config;
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

		ret = set_write_enable(dev_data, MX66_OSPI_DTR_MODE);
		if (ret != 0) {
			break;
		}

		k_event_clear(&dev_data->event_f,
			      OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

		if (len == (f_param->sector_size *
				f_param->num_of_sector)) {
			ret = erase_chip(dev);
			if (ret != 0) {
				break;
			}
			len -= f_param->sector_size *
			       f_param->num_of_sector;
			poll_timeout = MX_REG_READ_POLL_TIMEOUT_2_MIN;
		} else {
			ret = erase_sector(dev, addr);
			if (ret != 0) {
				break;
			}
			len -= f_param->sector_size;
			addr += f_param->sector_size;
			poll_timeout = MX_REG_READ_POLL_TIMEOUT_100_MS;
		}

		event = k_event_wait(&dev_data->event_f,
				     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST, false,
				     K_FOREVER);

		if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
			LOG_ERR("F-Erase: Incomplete Event [%d]", event);
			set_cs_pin(dev_data->ospi_handle, MX_CHIP_DE_ACTIVATE);
			ret = -EIO;
			break;
		}

		/** Read status with timeout */
		ret = poll_read_status_reg(dev_data, poll_timeout);
		if (ret != 0) {
			break;
		}

		/**Check Erase bit status */
		val = 0xFF;
		ret = read_sec_reg(dev_data, &val);
		if ((val & MX_SEC_ERASE_ERR_STATUS) || ret != 0) {
			return -EIO;
		}
	}

	/* Force De-Select */
	set_cs_pin(dev_data->ospi_handle, MX_CHIP_DE_ACTIVATE);

	k_sem_give(&dev_data->sem);

	return ret;
}

static int flash_mx66uw_ospi_init(const struct device *dev)
{
	int ret, dfs;
	uint8_t val;

	struct mx_flash_ospi_config *dev_cfg = (struct mx_flash_ospi_config *)dev->config;
	struct mx_flash_ospi_dev_data *dev_data = (struct mx_flash_ospi_dev_data *)dev->data;

	struct ospi_init init_config;

	/** DFS */
	dfs = get_dfs(dev_cfg->flash_param.write_block_size);
	if (dfs == 0) {
		return -EINVAL;
	}

	/** R-W FrameSize */
	dev_data->rw_dfs = dfs;

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

	init_config.xip_rxds_vl_en = DT_PROP(OSPI_CTRL_NODE, xip_rxds_vl_en);
	init_config.xip_wait_cycles = DT_PROP(OSPI_CTRL_NODE, xip_wait_cycles);

	memset(&dev_data->trans_conf, 0, sizeof(struct ospi_trans_config));

	/**Initial configurations */
	dev_data->trans_conf.frame_size = OSPI_DFS_BITS_8;
	dev_data->trans_conf.frame_format = OSPI_FRF_STANDRAD;
	dev_data->trans_conf.addr_len = 0;
	dev_data->trans_conf.wait_cycles = 0;
	dev_data->trans_conf.ddr_enable = OSPI_DDR_DISABLE;
	dev_data->trans_conf.ddr_ins_enable = OSPI_DDR_DISABLE;
	dev_data->trans_conf.inst_len = OSPI_INST_LENGTH_8_BITS;
	dev_data->trans_conf.trans_type = 0;
	dev_data->trans_conf.rx_ds_enable = 1;

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

	/* Enable write latch */
	ret = set_write_enable(dev_data, MX66_SPI_MODE);
	if (ret != 0) {
		return ret;
	}

	/* Update mode of Operation to DTR-OPI*/
	ret = set_dtr_ospi_mode(dev_data);
	if (ret != 0) {
		return ret;
	}

	/* Update config for DTR-OPI Mode */
	dev_data->trans_conf.ddr_enable = OSPI_DDR_ENABLE;
	dev_data->trans_conf.ddr_ins_enable = OSPI_DDR_ENABLE;
	dev_data->trans_conf.frame_format = OSPI_FRF_OCTAL;
	dev_data->trans_conf.frame_size = OSPI_DFS_BITS_16;
	dev_data->trans_conf.inst_len = OSPI_INST_LENGTH_16_BITS;
	dev_data->trans_conf.rx_ds_enable = 1;

	val = 0;
	ret = read_dev_id(dev_data, &val);
	if (val != MX_MANUFACTURER_ID) {
		return -ENODEV;
	}

	/* Enable Write latch */
	ret = set_write_enable(dev_data, MX66_OSPI_DTR_MODE);
	if (ret != 0) {
		return ret;
	}

	ret = update_dummy_cycle(dev_data, DT_PROP(OSPI_FLASH_NODE, def_wait_cycle));
	if (ret != 0) {
		return ret;
	}

	/* Read status with timeout */
	ret = poll_read_status_reg(dev_data, MX_REG_READ_POLL_TIMEOUT_100_MS);
	if (ret != 0) {
		return ret;
	}

	return ret;
}


#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_mx66uw_page_layout(const struct device *dev,
				const struct flash_pages_layout **layout,
				size_t *layout_size)
{
	struct mx_flash_ospi_config *dev_cfg = (struct mx_flash_ospi_config *) dev->config;

	*layout = &(dev_cfg->flash_layout);
	*layout_size = 1;
}
#endif

/* PINCTRL Definition Macro for Node */
PINCTRL_DT_DEFINE(OSPI_CTRL_NODE);

static const struct flash_parameters *flash_mx66uw_get_parameters(const struct device *dev)
{
	struct mx_flash_ospi_config *dev_cfg = (struct mx_flash_ospi_config *)dev->config;

	return &(dev_cfg->flash_param);
}

static const struct flash_driver_api flash_mx66uw_driver_api = {
	.read = flash_mx66uw_read,
	.write = flash_mx66uw_write,
	.erase = flash_mx66uw_erase,
	.get_parameters = flash_mx66uw_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mx66uw_page_layout,
#endif

};

static const struct mx_flash_ospi_config mx_flash_ospi_config = {
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

static struct mx_flash_ospi_dev_data mx_flash_ospi_data = {
	/**Add initial data.*/
};

static void flash_mx66uw_ospi_isr(const struct device *dev)
{
	struct mx_flash_ospi_dev_data *dev_data = (struct mx_flash_ospi_dev_data *)dev->data;

	/* ospi-irq handler */
	alif_hal_ospi_irq_handler(dev_data->ospi_handle);
}

DEVICE_DT_DEFINE(OSPI_FLASH_NODE, &flash_mx66uw_ospi_init, NULL, &mx_flash_ospi_data,
		 &mx_flash_ospi_config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		 &flash_mx66uw_driver_api);

static void flash_alif_ospi_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(OSPI_CTRL_NODE), DT_IRQ(OSPI_CTRL_NODE, priority),
			flash_mx66uw_ospi_isr, DEVICE_DT_GET(OSPI_FLASH_NODE), 0);
	irq_enable(DT_IRQN(OSPI_CTRL_NODE));
}
