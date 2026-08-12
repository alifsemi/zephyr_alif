/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT alif_secure_enclave_trng

#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <errno.h>
#include <se_service.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(alif_secure_enclave_trng, CONFIG_ENTROPY_LOG_LEVEL);

static int entropy_alif_rng_get_entropy(const struct device *dev,
					  uint8_t *buffer,
					  uint16_t length)
{
	int err;

	err = se_service_get_rnd_num(buffer, length);
	if (err) {
		LOG_ERR("Failed to get random number of length %d(err = %d)\n",
			length, err);
		return err;
	}
	return 0;
}

static int entropy_alif_rng_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static struct entropy_driver_api entropy_alif_rng_api_funcs = {
	.get_entropy = entropy_alif_rng_get_entropy
};

DEVICE_DT_INST_DEFINE(0,
			entropy_alif_rng_init, NULL,
			NULL, NULL,
			POST_KERNEL, CONFIG_ENTROPY_INIT_PRIORITY,
			&entropy_alif_rng_api_funcs);
