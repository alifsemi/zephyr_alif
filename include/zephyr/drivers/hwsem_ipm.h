/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_HWSEM_IPM_H_
#define ZEPHYR_INCLUDE_DRIVERS_HWSEM_IPM_H_

/**
 * @brief HWSEM IPM Interface
 * @defgroup hwsem_ipm_interface IPM Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @typedef hwsem_ipm_trylock_t
 * @brief API to try lock hardware semaphore (HWSEM).
 *
 * See @a hwsem_ipm_trylock() for argument definitions.
 *
 * @param hwsem_ipmdev - Driver instance
 * @param id - HWSEM represented by hwsem_ipmdev is tried to lock by
	       writing master ID 'id'.
 * @retval 0 On success.
 *         -EBUSY On HWSEM held by other cores.
 *         negative value on error.
 */
typedef int (*hwsem_ipm_trylock_t)(const struct device *hwsem_ipmdev,
				   uint32_t id);

/**
 * @typedef hwsem_ipm_lock_t
 * @brief API to lock hardware semaphore (HWSEM).
 *	  The API is blocked until HWSEM lock is acquired.
 *
 * See @a hwsem_ipm_lock() for argument definitions.
 *
 * @param hwsem_ipmdev - Driver instance
 * @param id - master ID that intends to lock HWSEM represented by
	       hwsem_ipmdev.
 * @retval 0 On success, negative value on error.
 */
typedef int (*hwsem_ipm_lock_t)(const struct device *hwsem_ipmdev,
				uint32_t id);

/**
 * @typedef hwsem_ipm_unlock_t
 * @brief API to unlock hardware semaphore (HWSEM).
 *	  The API is blocked until HWSEM lock is released.
 *
 * See @a hwsem_ipm_unlock() for argument definitions.
 *
 * @param hwsem_ipmdev - Driver instance
 * @param id - master ID that intends to unlock HWSEM represented by
	       hwsem_ipmdev.
 * @retval 0 On success, negative value on error.
 */
typedef int (*hwsem_ipm_unlock_t)(const struct device *hwsem_ipmdev,
				  uint32_t id);

__subsystem struct hwsem_ipm_driver_api {
	hwsem_ipm_trylock_t trylock;
	hwsem_ipm_lock_t lock;
	hwsem_ipm_unlock_t unlock;
};

__syscall int hwsem_trylock(const struct device *hwsem_ipmdev,
			    uint32_t id);

static inline int hwsem_trylock(const struct device *hwsem_ipmdev,
				uint32_t id)
{
	const struct hwsem_ipm_driver_api *api =
			(const struct hwsem_ipm_driver_api *)hwsem_ipmdev->api;

	if (api->trylock == NULL) {
		return -ENOSYS;
	}

	return api->trylock(hwsem_ipmdev, id);
}

__syscall int hwsem_lock(const struct device *hwsem_ipmdev,
			 uint32_t id);

static inline int hwsem_lock(const struct device *hwsem_ipmdev,
			     uint32_t id)
{
	const struct hwsem_ipm_driver_api *api =
			(const struct hwsem_ipm_driver_api *)hwsem_ipmdev->api;

	if (api->lock == NULL) {
		return -ENOSYS;
	}

	return api->lock(hwsem_ipmdev, id);
}

__syscall int hwsem_unlock(const struct device *hwsem_ipmdev,
			   uint32_t id);

static inline int hwsem_unlock(const struct device *hwsem_ipmdev,
			       uint32_t id)
{
	const struct hwsem_ipm_driver_api *api =
			(const struct hwsem_ipm_driver_api *)hwsem_ipmdev->api;

	if (api->unlock == NULL) {
		return -ENOSYS;
	}

	return api->unlock(hwsem_ipmdev, id);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_HWSEM_IPM_H_ */
