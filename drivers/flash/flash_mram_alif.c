/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT alif_mram_flash_controller

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <string.h>
#include <cmsis_core.h>
#include <errno.h>
#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_mram_alif);

#define SOC_NV_FLASH_NODE      DT_NODELABEL(mram_storage)
#define FLASH_MRAM_BASE_OFFSET DT_REG_ADDR(SOC_NV_FLASH_NODE)
#define FLASH_MRAM_ERASE_UNIT  DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)
#define FLASH_MRAM_PROG_UNIT   DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_MRAM_FLASH_SIZE  DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_MRAM_ERASE_VALUE 0x0
#define FLASH_MRAM_PAGE_COUNT  (FLASH_MRAM_FLASH_SIZE / FLASH_MRAM_ERASE_UNIT)

#if (FLASH_MRAM_ERASE_UNIT % FLASH_MRAM_PROG_UNIT)
#error "Erase unit must be a multiple of program unit"
#endif

#define MRAM_FLASH(offset) ((volatile uint8_t *)FLASH_MRAM_BASE_OFFSET + (offset))

/* MRAM(On-Chip NVM) Sector Size: 16-Byte(128-bit) */
#define MRAM_UNIT_SECTOR_SIZE    (FLASH_MRAM_PROG_UNIT)
#define MRAM_ADDR_ALIGN_MASK     (uint32_t)(0xFFFFFFF0)

struct mram_flash_data {
	/* mram flash write protection is managed in software. */
	struct k_sem lock;
};

static const struct flash_parameters flash_mram_parameters = {
	.write_block_size = FLASH_MRAM_PROG_UNIT,
	.erase_value = FLASH_MRAM_ERASE_VALUE
};

/**
 * @brief write 16 bytes of data into MRAM
 *
 * @param dst Pointer containing MRAM address where data is written.
 * @param src Pointer to source containing data to be written.
 *
 * @return 0 if successful.
 * @return -EINVAL if dst is not 16 bytes aligned address.
 */
static int mram_write_16bytes(volatile void *dst, const void *src)
{
	if ((uintptr_t)dst & 0xF) {
		LOG_ERR("Address %p must be %d bytes aligned for MRAM write\n", (void *)dst,
			MRAM_UNIT_SECTOR_SIZE);
		return -EINVAL;
	}

	const uint32_t *src32 = src;
	volatile uint32_t *dst32 = dst;

	__disable_irq();

	dst32[0] = UNALIGNED_GET(&src32[0]);
	dst32[1] = UNALIGNED_GET(&src32[1]);
	dst32[2] = UNALIGNED_GET(&src32[2]);
	dst32[3] = UNALIGNED_GET(&src32[3]);

	__enable_irq();

	return 0;
}

/**
 * @brief erase 16 bytes of MRAM
 *
 * @param dst Pointer containing MRAM address for erase.
 *
 * @return 0 if successful.
 * @return -EINVAL if dst is not 16 bytes aligned address.
 */
static int mram_erase_16bytes(volatile void *dst)
{
	if ((uintptr_t)dst & 0xF) {
		LOG_ERR("Address %p must be %d bytes aligned for MRAM write\n", (void *)dst,
			MRAM_UNIT_SECTOR_SIZE);
		return -EINVAL;
	}

	volatile uint32_t *dst32 = dst;

	__disable_irq();

	dst32[0] = 0;
	dst32[1] = 0;
	dst32[2] = 0;
	dst32[3] = 0;

	__enable_irq();

	return 0;
}

/**
 * @brief check if offset and len are in valid range.
 *
 * @param dev Pointer to device driver instance
 * @param offset Flash offset address for read/write operation.
 * @param len Number of bytes to read/write from offset address.
 *
 * @return 1 if range is valid, 0 otherwise.
 */
static int flash_range_is_valid(const struct device *dev, off_t offset,
				size_t len)
{
	ARG_UNUSED(dev);
	if (((offset + len) > FLASH_MRAM_FLASH_SIZE) || (offset < 0x0)) {
		return 0;
	}
	return 1;
}

/**
 * @brief read aligned data from the MRAM from the given address.
 *
 * @param dst Pointer to the void buffer to store read data.
 * @param src Pointer to volatile containing MRAM address at which read starts.
 * @param len Number of bytes to read from MRAM address.
 */
static void memcpy_with_aligned_reads(void *dst, const volatile void *src, size_t len)
{
	const volatile uint8_t *src8 = src;
	uint8_t *dst8 = dst;
	size_t remaining = len;

	/* align MRAM source to 32-bit */
	while (remaining && ((uintptr_t)src8 & 0x3)) {
		*dst8++ = *src8++;
		remaining--;
	}

	/* copy 32-bit data */
	const volatile uint32_t *src32 = (const volatile uint32_t *)src8;
	uint32_t *dst32 = (uint32_t *)dst8;

	while (remaining >= 4) {
		uint32_t mram_word = *src32++;

		UNALIGNED_PUT(mram_word, dst32);

		dst32++;
		remaining -= 4;
	}

	/* copy remaining bytes. */
	src8 = (const volatile uint8_t *)src32;
	dst8 = (uint8_t *)dst32;

	while (remaining--) {
		*dst8++ = *src8++;
	}
}

/**
 * @brief read data from the flash from the given address.
 *
 * @param dev Pointer to device driver instance
 * @param offset Flash offset address at which read starts.
 * @param data Pointer to the uint8_t buffer to store read data.
 * @param len Number of bytes to read from offset address.
 *
 * @return 0 if successful.
 * @return -EINVAL if offset or len is out of range.
 * @return -EACCES if unable to take semaphore.
 */
static int flash_mram_read(const struct device *dev, const off_t offset,
			   void *data, const size_t len)
{
	struct mram_flash_data *dev_data = dev->data;

	if (!flash_range_is_valid(dev, offset, len)) {
		LOG_ERR("mram_read: Invalid range offset: %ld len: %d\n",
			(long)offset, len);
		return -EINVAL;
	}
	if (k_sem_take(&dev_data->lock, K_FOREVER)) {
		LOG_ERR("mram_read: unable to take semaphore\n");
		return -EACCES;
	}

	/* Reading from MRAM does not have 16 bytes alignment or */
	/* the multiple of 16 read size constraints */

	const volatile void *src = MRAM_FLASH(offset);
	/* read aligned data from the MRAM */
	memcpy_with_aligned_reads(data, src, len);

	k_sem_give(&dev_data->lock);

	return 0;
}

/**
 * @brief write data into the flash from the given address.
 *
 * @param dev Pointer to device driver instance.
 * @param offset Flash offset address at which write starts.
 * @param data Pointer to the uint8_t buffer containing data for write.
 * @param len Number of bytes to write from offset address.
 *
 * @return 0 if successful.
 * @return -EINVAL if offset or len is out of range.
 * @return -EACCES if unable to take semaphore.
 */
static int flash_mram_write(const struct device *dev, const off_t offset,
			     const void *data, const size_t len)
{
	struct mram_flash_data *dev_data = dev->data;

	if (!flash_range_is_valid(dev, offset, len)) {
		LOG_ERR("mram_write: Invalid range offset: %ld len: %d\n",
			(long)offset, len);
		return -EINVAL;
	}

	if (k_sem_take(&dev_data->lock, K_FOREVER)) {
		LOG_ERR("mram_write: unable to take semaphore\n");
		return -EACCES;
	}

	/* add MRAM Base-address to offset. */
	uint32_t addr = (uint32_t) MRAM_FLASH(offset);

	/* check address with aligned to 16-Bytes.*/
	uint32_t aligned_addr   = addr & MRAM_ADDR_ALIGN_MASK;
	volatile uint8_t *p_aligned_addr = (volatile uint8_t *) aligned_addr;
	const uint8_t *p_data   = data;
	uint32_t count          = len;

	/* use temporary buffer to store data in case of unaligned memory.*/
	uint8_t temp_buff[MRAM_UNIT_SECTOR_SIZE] = {0}; /* 16 bytes */

	/* is MRAM address aligned to 16-Byte? */
	if (addr != aligned_addr) {
		/* unaligned MRAM address:
		 *  - make it to nearest aligned 16-Byte address,
		 *     by writing only unaligned bytes.
		 */

		uint8_t unaligned_offset = addr & (~MRAM_ADDR_ALIGN_MASK);
		uint8_t unaligned_bytes  = MRAM_UNIT_SECTOR_SIZE - unaligned_offset;

		/* is unaligned bytes more than remaining count? */
		if (unaligned_bytes > count) {
			/* then take only remaining count. */
			unaligned_bytes = count;
		}

		/* copy original one sector data from MRAM address to buffer. */
		memcpy_with_aligned_reads(temp_buff, p_aligned_addr, MRAM_UNIT_SECTOR_SIZE);

		/* overwrite buffer with new data as per offset and unaligned bytes. */
		memcpy(temp_buff + unaligned_offset, p_data, unaligned_bytes);

		/* now, copy 128bit from buffer to MRAM. */
		mram_write_16bytes(p_aligned_addr, temp_buff);

		p_aligned_addr += MRAM_UNIT_SECTOR_SIZE;
		p_data         += unaligned_bytes;
		count          -= unaligned_bytes;
	}

	uint32_t sector_cnt    = count / MRAM_UNIT_SECTOR_SIZE;
	uint8_t unaligned_cnt  = count % MRAM_UNIT_SECTOR_SIZE;

	/* program 128-bit to absolute sector. */
	while (sector_cnt--) {
		/* as MRAM address is 16-byte aligned,
		 * directly copy 128bit from source-data to MRAM.
		 */
		mram_write_16bytes(p_aligned_addr, p_data);

		p_aligned_addr += MRAM_UNIT_SECTOR_SIZE;
		p_data         += MRAM_UNIT_SECTOR_SIZE;
	}

	/* program remaining unaligned data. */
	if (unaligned_cnt) {
		/* copy original one sector data from MRAM address to buffer.*/
		memcpy_with_aligned_reads(temp_buff, p_aligned_addr, MRAM_UNIT_SECTOR_SIZE);

		/* overwrite buffer with new data as per remaining unaligned count.*/
		memcpy(temp_buff, p_data, unaligned_cnt);

		/* now, copy 128bit from buffer to MRAM. */
		mram_write_16bytes(p_aligned_addr, temp_buff);
	}

	k_sem_give(&dev_data->lock);

	return 0;
}

/**
 * @brief erase data of FLASH_MRAM_ERASE_UNIT bytes from the MRAM flash.
 *
 * @param unit Address to erase FLASH_MRAM_ERASE_UNIT bytes.
 *             The erase value is defined by FLASH_MRAM_ERASE_VALUE.
 */
static void mram_unit_erase(const uint32_t unit)
{
	const off_t unit_addr = (unit * FLASH_MRAM_ERASE_UNIT);
	uint32_t i;

	for (i = unit_addr; i < (unit_addr + FLASH_MRAM_ERASE_UNIT);
	    i += FLASH_MRAM_PROG_UNIT) {
		/* Erase 16 bytes of MRAM data */
		mram_erase_16bytes(MRAM_FLASH(i));
	}
}

/**
 * @brief erase data from the flash at the given address.
 *
 * @param dev Pointer to device driver instance.
 * @param offset Flash offset address at which erase starts.
 * @param len Number of bytes to erase from offset address.
 *
 * @return 0 if successful.
 * @return -EINVAL if offset or len is out of range.
 * @return -EACCES if unable to take semaphore.
 */
static int flash_mram_erase(const struct device *dev, off_t offset,
			    size_t len)
{
	struct mram_flash_data *dev_data = dev->data;
	uint32_t mram_unit_start;
	uint32_t i;

	if (!flash_range_is_valid(dev, offset, len)) {
		LOG_ERR("mram_erase: Invalid range offset: %ld len :%d\n",
			(long)offset, len);
		return -EINVAL;
	}
	if ((offset % FLASH_MRAM_ERASE_UNIT) ||
	   (len % FLASH_MRAM_ERASE_UNIT)) {
		LOG_ERR("mram_erase: offset %ld and len %d "
			"must be multiple of %d\n",
			(long)offset, len, FLASH_MRAM_ERASE_UNIT);
		return -EINVAL;
	}
	mram_unit_start = offset / FLASH_MRAM_ERASE_UNIT;
	if (k_sem_take(&dev_data->lock, K_FOREVER)) {
		LOG_ERR("mram_erase: unable to take semaphore\n");
		return -EACCES;
	}
	for (i = 0 ; i < len / FLASH_MRAM_ERASE_UNIT ; ++i) {
		mram_unit_erase(mram_unit_start + i);
	}
	k_sem_give(&dev_data->lock);

	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_mram_pages_layout = {
	.pages_count = FLASH_MRAM_PAGE_COUNT,
	.pages_size = FLASH_MRAM_ERASE_UNIT,
};

static void flash_mram_page_layout(const struct device *dev,
				  const struct flash_pages_layout **layout,
				  size_t *layout_size)
{
	*layout = &flash_mram_pages_layout;
	*layout_size = 1;
}
#endif

static const struct flash_parameters *
flash_mram_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_mram_parameters;
}

static const struct flash_driver_api flash_mram_api = {
	.read = flash_mram_read,
	.write = flash_mram_write,
	.erase = flash_mram_erase,
	.get_parameters = flash_mram_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mram_page_layout,
#endif
};

static int flash_mram_init(const struct device *dev)
{
	struct mram_flash_data *dev_data = dev->data;

	k_sem_init(&dev_data->lock, 1, 1);
	/* NOTE: Uncomment below line to erase storage parition of MRAM */
	/* flash_mram_erase(dev, 0x0, FLASH_MRAM_FLASH_SIZE); */
	return 0;
}
static struct mram_flash_data data;

DEVICE_DT_INST_DEFINE(0, flash_mram_init, NULL,
			&data, NULL,
			POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
			&flash_mram_api);
