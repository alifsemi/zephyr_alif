/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CRC_H_
#define ZEPHYR_INCLUDE_DRIVERS_CRC_H_

#include <zephyr/device.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

struct crc_params {
	const void	*data_in;	/* Pointer to Input buffer	 */
	uint32_t	len;		/* Total length of Input buffer  */
	uint32_t	*data_out;	/* Pointer to CRC Output	 */
	bool		reflect;	/* CRC reflect			 */
	bool		invert;		/* CRC Invert			 */
	bool		bit_swap;	/* CRC bit swap			 */
	bool		byte_swap;	/* CRC byte swap		 */
	bool		custom_poly;	/* CRC custom polynomial	 */
};

typedef int (*crc_api_compute_t)(const struct device *dev, struct crc_params *cfg);
typedef int (*crc_api_set_seed_t)(const struct device *dev, uint32_t seed_value);
typedef int (*crc_api_set_polynomial_t)(const struct device *dev, uint32_t polynomial);

__subsystem struct crc_driver_api {
	crc_api_compute_t		compute;
	crc_api_set_seed_t		set_seed;
	crc_api_set_polynomial_t	set_polynomial;
};

/**
 * @brief Start crc compute.
 * calculate the CRC result for 8 bit 16 bit and 32 bit CRC algorithm.
 * @param dev	: Pointer to the device structure for the driver instance.
 * @param params : pointer to crc_params which has input buffer, length of
 *		the input buffer, pointer to crc output, reflect, invert, bit
 *		swap, byte swap and custom polynomial parameters
 * @retval 0 If successful.
 * @retval -ENOSYS   If the driver does not implement this function.
 */
__syscall int crc_compute(const struct device *dev, struct crc_params *params);

static inline int z_impl_crc_compute(const struct device *dev, struct crc_params *params)
{
	const struct crc_driver_api *api =
			(struct crc_driver_api *)dev->api;

	if (api->compute == NULL) {
		return -ENOSYS;
	}

	return api->compute(dev, params);
}

/**
 * @brief Set crc seed value.
 * Set seed value depending on whether the data is 8 bit or 16 or 32 bit
 * @param dev	: Pointer to the device structure for the driver instance.
 * @param seed_value : Write a seed value for the desired algorithm
 * @retval 0 If successful.
 * @retval -ENOSYS   If the driver does not implement this function.
 */
__syscall int crc_set_seed(const struct device *dev, uint32_t seed_value);

static inline int z_impl_crc_set_seed(const struct device *dev, uint32_t seed_value)
{
	const struct crc_driver_api *api =
			(struct crc_driver_api *)dev->api;

	if (api->set_seed == NULL) {
		return -ENOSYS;
	}

	return api->set_seed(dev, seed_value);
}

/**
 * @brief Add polynomial value
 * Custom polynomials can only be used for 32-bit CRC algorithms.
 * @param dev	: Pointer to the device structure for the driver instance.
 * @param polynomial : if custom crc algorithm is used, write a polynomial
 *		value for 32-bit algorithm
 * @retval 0 If successful.
 * @retval -ENOSYS   If the driver does not implement this function.
 */
__syscall int crc_set_polynomial(const struct device *dev, uint32_t polynomial);

static inline int z_impl_crc_set_polynomial(const struct device *dev, uint32_t polynomial)
{
	const struct crc_driver_api *api =
			(struct crc_driver_api *)dev->api;

	if (api->set_polynomial == NULL) {
		return -ENOSYS;
	}

	return api->set_polynomial(dev, polynomial);
}

#include <zephyr/syscalls/crc.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CRC_H_ */
