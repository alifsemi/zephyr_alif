/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_alif_crc

#include <errno.h>
#include <stdint.h>
#include <zephyr/drivers/crc.h>
#include <zephyr/kernel.h>
#include "alif_crc_reg.h"
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#define CONF_CRC_INIT_PRIORITY	40

LOG_MODULE_REGISTER(alif_crc, LOG_LEVEL_INF);

/**
 * @fn		crc_bit_reflect(uint32_t input)
 * @brief	Reflect the CRC 32 bit output
 * @param[in]	input	: 32 bit CRC output
 * @return	result of reflected CRC 32 bit output
 */
static uint32_t crc_bit_reflect(uint32_t input)
{
	uint32_t res = 0;
	uint32_t i, bit;

	for (i = 0; i < 32; i++) {
		bit = (input >> i) & 1;
		bit = bit << (32 - (i + 1));
		res |= bit;
	}

	return res;
}

/**
 * @fn		uint32_t crc_calculate_unaligned(uint32_t key,const
 *						 uint8_t *input,
						 uint32_t length,
						 uint32_t poly)
 * @brief	To calculate the CRC result for unaligned data
			1. It will take the aligned data for CRC result from the hardware.
			2. Unaligned input data and its length.
			3. If the algorithm is 32 bit CRC then it will take the standard
				32 bit CRC Polynomial
			4. If the algorithm is 32 bit Custom CRC polynomial , it will take
				the polynomial entered from the user.
 * @param[in]  dev   : pointer to Runtime device structure
 * @param[in]  crc   : Output of aligned data for CRC from the hardware
 * @param[in]  input : unaligned input data
 * @param[in]  length: length of unaligned data
 * @param[in]  poly  : Standard polynomial or the user entered polynomial
					depending upon the CRC algorithm
 * @return	Calculated CRC output for unaligned data
 */
static uint32_t crc_calculate_unaligned(uint32_t crc, const uint8_t *input,
				 uint32_t length, uint32_t poly)
{
	uint32_t check_bit, polynomial;
	uint8_t  data;
	uint32_t byte_idx, bit_idx;

	/* Store the reflected polynomial  */
	polynomial = crc_bit_reflect(poly);

	for (byte_idx  = 0; byte_idx  < length; byte_idx++) {
		data = input[byte_idx];
		for (bit_idx  = 0; bit_idx  < 8; bit_idx++) {
			check_bit = (crc ^ data) & 1;
			crc >>= 1;

			if (check_bit) {
				crc = crc ^ polynomial;
			}
			data >>= 1;
		}
	}
	return ~crc;
}

/**
 * @fn		void crc_calculate_8bit(const struct device *dev, crc_params *params)
 * @brief	Calculate the CRC output  for 8 bit CRC algorithm.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   params  : pointer to crc_params structure which has length of the
			  input buffer
 * @return	 None
 */
static void crc_calculate_8bit(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	for (int count = 0; count < params->len ; count++) {
		sys_write8(((const uint8_t *)params->data_in)[count], reg_base + CRC_DATA_IN_8_0);
	}

	/* Store the CRC output  */
	*params->data_out = sys_read32(reg_base + CRC_OUT);
}

/**
 * @fn		void crc_calculate_16bit(const struct device *dev, crc_params *params)
 * @brief	Calculate the CRC output  for 16 bit CRC algorithm.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   params  : pointer to crc_params structure which has length of the
			  input buffer
 * @return	 None
 */
static void crc_calculate_16bit(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	for (int count = 0; count < params->len ; count++) {
		sys_write8(((const uint8_t *)params->data_in)[count], reg_base + CRC_DATA_IN_8_0);
	}

	/* Store the CRC output  */
	*params->data_out =  sys_read32(reg_base + CRC_OUT);
}

/**
 * @fn		void crc_calculate_32bit(const struct device *dev, crc_params *params)
 * @brief	Calculate the CRC output  for 32 bit CRC algorithm.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   params  : pointer to crc_params structure which has length of the
			  input buffer
 * @return	 None
 */
static void crc_calculate_32bit(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	const uint32_t *data32;
	uint32_t value;
	uint32_t aligned_length = params->len - (params->len % 4);

	data32 = (const uint32_t *)params->data_in;

	for (int count = 0; count < aligned_length / 4 ; count++) {
		value = *(data32++);
		sys_write32(value, reg_base + CRC_DATA_IN_32_0);
	}

	/* Store the CRC output  */
	*params->data_out =  sys_read32(reg_base + CRC_OUT);
}

/**
 * @fn		void crc_calculate_32bit_unaligned_sw(const struct device *dev,
		crc_params *params)
 * @brief	Calculate the 32bit CRC output for the unaligned part.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   params  : pointer to crc_params structure which has reflect, invert
			  and custom polynomial parameters
 * @return	 None
 */
static void crc_calculate_32bit_unaligned_sw(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t  custom;
	uint32_t unaligned_length = (params->len % 4);
	uint32_t aligned_length = params->len - (params->len % 4);
	const uint8_t *input = (const uint8_t *)params->data_in + aligned_length;

	uint32_t crc_algo = (sys_read32(reg_base + CRC_CONTROL) & CRC_ALGO_MASK);

	if (unaligned_length > 0) {
		/* Check for the custom polynomial bit   */
		if (params->custom_poly) {
			/* add the user polynomial  */
			custom = sys_read32(reg_base + CRC_POLY_CUSTOM);
		} else {
			if (crc_algo == CRC_32) {
				/* add the CRC32 standard polynomial */
				custom = CRC_32_STANDARD_POLY;
			} else {
				/* add the CRC32C standard polynomial */
				custom = CRC_32C_STANDARD_POLY;
			}
		}

		if (params->invert) {
			*params->data_out = ~(*params->data_out);
		}

		if (!(params->reflect)) {
			*params->data_out = crc_bit_reflect(*params->data_out);
		}

		/* Calculate the CRC for unaligned data  */
		*params->data_out = crc_calculate_unaligned(*params->data_out,
							    input,
							    unaligned_length,
							    custom);

		if (!(params->reflect)) {
			*params->data_out = crc_bit_reflect(*params->data_out);
		}

		if (!(params->invert)) {
			*params->data_out = ~(*params->data_out);
		}
	}
}

/**
 * @fn		void crc_enable_crc8_ccitt(const struct device *dev)
 * @brief	Enable CRC-8-CCITT algorithm and size.
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static inline void crc_enable_crc8_ccitt(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val = sys_read32(reg_base + CRC_CONTROL);

	val &= ~(CRC_ALGO_MASK | CRC_ALGO_SIZE_MASK);
	val |= (CRC_8_CCITT | CRC_ALGO_8_BIT_SIZE | CRC_INIT_BIT);

	sys_write32(val, reg_base + CRC_CONTROL);
}

/**
 * @fn		void crc_enable_crc16(const struct device *dev)
 * @brief	Enable CRC-16 algorithm and size.
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static inline void crc_enable_crc16(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val = sys_read32(reg_base + CRC_CONTROL);

	val &= ~(CRC_ALGO_MASK | CRC_ALGO_SIZE_MASK);
	val |= (CRC_16 | CRC_ALGO_16_BIT_SIZE | CRC_INIT_BIT);

	sys_write32(val, reg_base + CRC_CONTROL);
}

/**
 * @fn		void crc_enable_crc16_ccitt(const struct device *dev)
 * @brief	Enable CRC-16-CCITT algorithm and size.
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static inline void crc_enable_crc16_ccitt(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val = sys_read32(reg_base + CRC_CONTROL);

	val &= ~(CRC_ALGO_MASK | CRC_ALGO_SIZE_MASK);
	val |= (CRC_16_CCITT | CRC_ALGO_16_BIT_SIZE | CRC_INIT_BIT);

	sys_write32(val, reg_base + CRC_CONTROL);
}

/**
 * @fn		void crc_enable_crc32(const struct device *dev)
 * @brief	Enable CRC32 algorithm and size.
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static inline void crc_enable_crc32(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val = sys_read32(reg_base + CRC_CONTROL);

	val &= ~(CRC_ALGO_MASK | CRC_ALGO_SIZE_MASK);
	val |= (CRC_32 | CRC_ALGO_32_BIT_SIZE | CRC_INIT_BIT);

	sys_write32(val, reg_base + CRC_CONTROL);
}

/**
 * @fn		void crc_enable_32c(const struct device *dev)
 * @brief	Enable CRC32C algorithm and size.
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static inline void crc_enable_32c(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val = sys_read32(reg_base + CRC_CONTROL);

	val &= ~(CRC_ALGO_MASK | CRC_ALGO_SIZE_MASK);
	val |= (CRC_32C | CRC_ALGO_32_BIT_SIZE | CRC_INIT_BIT);

	sys_write32(val, reg_base + CRC_CONTROL);
}

/**
 * @fn		void crc_params_init(const struct device *dev,uint32_t seed_value)
 * @brief	Initailizing crc parameters
 * @param[in]   dev	 : pointer to Runtime device structure
 * @param[in]   params   : pointer to crc_params which has input buffer, length of
			   the input buffer, pointer to crc output, reflect, invert,
			   bit swap, byte swap and custom polynomial parameters
 * @return	None
 */
static void crc_params_init(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val = sys_read32(reg_base + CRC_CONTROL);

	val &= ~(CRC_REFLECT | CRC_INVERT | CRC_BIT_SWAP | CRC_BYTE_SWAP | CRC_CUSTOM_POLY);

	if (params->reflect) {
		val |= CRC_REFLECT;
	}
	if (params->invert) {
		val |= CRC_INVERT;
	}
	if (params->bit_swap) {
		val |= CRC_BIT_SWAP;
	}
	if (params->byte_swap) {
		val |= CRC_BYTE_SWAP;
	}
	if (params->custom_poly) {
		val |= CRC_CUSTOM_POLY;
	}

	val |= CRC_INIT_BIT;

	sys_write32(val, reg_base + CRC_CONTROL);
}

/**
 * @fn		int alif_crc_compute(const struct device *dev, crc_params *params)
 * @brief	1.calculate the CRC result for CRC-8-CCITT, CRC-16, CRC-16-CCITT
 *		CRC-32 and CRC-32C CRC algorithm.
		2.For 8 bit and 16 bit CRC algorithm our hardware can able to
		calculate the CRC result for both aligned and unaligned CRC input
		data by loading the CRC inputs in DATA_IN_8 bit register.
		3.For 32 bit CRC our hardware will support for aligned data to
		calculate the CRC Result.
		4.For unaligned data CRC_calculate_Unaligned function will calculate
		the CRC result for unaligned CRC input
		5. In CRC_calculate_Unaligned function load the aligned CRC result from
		the hardware ,unaligned CRC input,length of unaligned input data and
		the polynomial for the 32 bit CRC
 * @param[in]  dev	: pointer to Runtime device structure
 * @param[in]  params   : pointer to crc_params which has input buffer, length of the
			  input buffer, pointer to crc output, reflect, invert, bit
			  swap, byte swap and custom polynomial parameters
 * @return	 None
 */
static int alif_crc_compute(const struct device *dev, struct crc_params *params)
{
	struct crc_data *data = dev->data;

	if (params->data_in == NULL || params->data_out == NULL || params->len == 0) {
		return -EINVAL;
	}

	/* Initializing crc parameters  */
	crc_params_init(dev, params);

	switch (data->crc_algo) {
		/* CRC-8-CCITT algorithm */
	case CRC_ALGO_CRC8_CCITT:

		/* Enable CRC-8-CCITT algorithm */
		crc_enable_crc8_ccitt(dev);

		/* Calculate the CRC output for CRC-8-CCITT algorithm  */
		crc_calculate_8bit(dev, params);

		break;

		/* CRC-16 algorithm */
	case CRC_ALGO_CRC16:

		/* Enable CRC-16 algorithm */
		crc_enable_crc16(dev);

		/* Calculate the CRC output for CRC-16 algorithm  */
		crc_calculate_16bit(dev, params);

		break;

		/* CRC-16-CCITT algorithm */
	case CRC_ALGO_CRC16_CCITT:

		/* Enable CRC-16-CCITT algorithm */
		crc_enable_crc16_ccitt(dev);

		/* Calculate the CRC output for CRC-16-CCITT algorithm  */
		crc_calculate_16bit(dev, params);

		break;

		/* CRC-32 algorithm */
	case CRC_ALGO_CRC32:

		/* Enable CRC-32 algorithm */
		crc_enable_crc32(dev);

		/* Calculate the CRC output for CRC-32 algorithm  */
		crc_calculate_32bit(dev, params);

		/* Handle remaining unaligned bytes in software */
		crc_calculate_32bit_unaligned_sw(dev, params);

		break;

		/* CRC-32C algorithm */
	case CRC_ALGO_CRC32C:

		/* Enable CRC-32C algorithm */
		crc_enable_32c(dev);

		/* Calculate the CRC output for CRC-32C algorithm  */
		crc_calculate_32bit(dev, params);

		/* Handle remaining unaligned bytes in software */
		crc_calculate_32bit_unaligned_sw(dev, params);

		break;

	default:
		LOG_ERR("Unsupported CRC algorithm");
		return -EINVAL;
	}
	return 0;
}

/**
 * @fn		void alif_crc_set_seed(const struct device *dev,uint32_t seed_value)
 * @brief	Set crc seed value
 * @param[in]   seed_value : Seed value depending on whether the data is 8 bit
			    or 16 or 32 bit
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static int alif_crc_set_seed(const struct device *dev, uint32_t seed_value)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val = sys_read32(reg_base + CRC_CONTROL);

	sys_write32(seed_value, reg_base + CRC_SEED);

	val |= CRC_INIT_BIT;

	sys_write32(val, reg_base + CRC_CONTROL);

	return 0;
}

/**
 * @fn		void alif_crc_set_polynomial(const struct device *dev,uint32_t polynomial)
 * @brief	add polynomial value
 * @param[in]   polynomial : Polynomial data for 8 bit or 16 or 32 bit
 * @param[in]   dev	   : pointer to Runtime device structure
 * @return	None
 */
static int alif_crc_set_polynomial(const struct device *dev, uint32_t polynomial)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val = sys_read32(reg_base + CRC_CONTROL);

	sys_write32(polynomial, reg_base + CRC_POLY_CUSTOM);

	val |= CRC_INIT_BIT;

	sys_write32(val, reg_base + CRC_CONTROL);

	return 0;
}

static const struct crc_driver_api crc_api_funcs = {
		.compute = alif_crc_compute,
		.set_seed = alif_crc_set_seed,
		.set_polynomial = alif_crc_set_polynomial,
};

/* Init function  */
static int crc_initialize(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

/********************** Device Definition per instance Macros. ***********************/

#define CRC_INIT(n)									\
	static struct crc_data data##n = {						\
			.crc_algo = DT_INST_ENUM_IDX(n, crc_algo)			\
	};										\
	static const struct crc_config config_##n = {					\
			DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),				\
	 };										\
	DEVICE_DT_INST_DEFINE(n, crc_initialize, NULL, &data##n,			\
						  &config_##n, POST_KERNEL,		\
						  CONF_CRC_INIT_PRIORITY, &crc_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(CRC_INIT)
