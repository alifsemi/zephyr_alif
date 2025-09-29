/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_COMPARATOR_ALIF_CMP_H_
#define ZEPHYR_INCLUDE_DRIVERS_COMPARATOR_ALIF_CMP_H_

#include <zephyr/device.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*comparator_callback) (const struct device *dev, uint8_t event);

struct cmp_params {
	uint8_t	filter_taps;
	uint8_t	prescalar;
	bool	polarity;
	comparator_callback callback;
};

typedef int (*cmp_start)(const struct device *dev);
typedef int (*cmp_setup)(const struct device *dev, struct cmp_params *setup);
typedef void (*cmp_windowing)(const struct device *dev, uint32_t window_ctrl);

struct cmp_driver_api {
	cmp_start			start;
	cmp_setup			setup;
	cmp_windowing		windowing;
};

/**
 * @brief Start comparator comparison.
 * compare the positive and negative analog input.
 * @param dev	: Pointer to the device structure for the driver instance.
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
__syscall int cmp_start_compare(const struct device *dev);

static inline int cmp_start_compare(const struct device *dev)
{
	const struct cmp_driver_api *api =
			(struct cmp_driver_api *)dev->api;

	return api->start(dev);
}

/**
 * @brief Configure the comparator
 * Configure the comparator like polarity, filer taps and windowing.
 * @param dev	: Pointer to the device structure for the driver instance.
 * @param params	: Pointer to the parameters structure.
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
__syscall int cmp_configure(const struct device *dev, struct cmp_params *params);

static inline int cmp_configure(const struct device *dev, struct cmp_params *params)
{
	const struct cmp_driver_api *api =
			(struct cmp_driver_api *)dev->api;

	return api->setup(dev, params);
}

/**
 * @brief	Comparator window control
 * Set the windowing control for the driver.
 * @param dev	: Pointer to the device structure for the driver instance.
 * @param window_ctrl : Enable or disable window control (bitfield)
 */
__syscall void cmp_window_ctrl(const struct device *dev, uint32_t window_ctrl);

static inline void cmp_window_ctrl(const struct device *dev, uint32_t window_ctrl)
{
	const struct cmp_driver_api *api =
			(struct cmp_driver_api *)dev->api;

	return api->windowing(dev, window_ctrl);
}


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_COMPARATR_ALIF_CMP_H_ */

