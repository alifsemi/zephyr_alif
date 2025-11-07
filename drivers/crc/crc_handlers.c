/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/crc.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/kernel.h>

static inline int z_vrfy_crc_compute(const struct device *dev,
				struct crc_params *params)
{
	struct crc_params params_copy;

	K_OOPS(K_SYSCALL_DRIVER_CRC(dev, compute));
	K_OOPS(k_usermode_from_copy(&params_copy,
				(struct crc_params *)params,
				sizeof(struct crc_params)));

	K_OOPS(K_SYSCALL_MEMORY_READ(params_copy.data_in, params_copy.len));
	K_OOPS(K_SYSCALL_MEMORY_WRITE(params_copy.data_out, sizeof(uint32_t)));

	return z_impl_crc_compute((const struct device *)dev,
				&params_copy);
}
#include <zephyr/syscalls/crc_compute_mrsh.c>

static inline int z_vrfy_crc_set_seed(const struct device *dev,
				uint32_t seed_value)
{
	K_OOPS(K_SYSCALL_DRIVER_CRC(dev, set_seed));

	return z_impl_crc_set_seed((const struct device *)dev,
				seed_value);
}
#include <zephyr/syscalls/crc_set_seed_mrsh.c>

static inline int z_vrfy_crc_set_polynomial(const struct device *dev,
					uint32_t polynomial)
{
	K_OOPS(K_SYSCALL_DRIVER_CRC(dev, set_polynomial));

	return z_impl_crc_set_polynomial((const struct device *)dev,
				polynomial);
}
#include <zephyr/syscalls/crc_set_polynomial_mrsh.c>
