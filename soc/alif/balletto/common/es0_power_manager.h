/*
 * Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __ES0_POWER_MANAGER_H__
#define __ES0_POWER_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
/*
 * This class is taking care of power modes of the available system cores.
 * It will also take care of the users of a specific core and when last user
 * stops using the core, it will be shutdown to save power.
 */

enum core_error_t {
	ALIF_PM_ERROR_COUNTER_CORRUPTED = -2,
	ALIF_PM_ERROR_FAILED = -1,
	ALIF_PM_ERROR_NONE = 0
};

#define BD_ADDRESS_LENGTH 6

/**
 * @brief Enable or disable HPA mode for controller RF.
 * Note: Needs to be set before take_es0_into_use() is called.
 * If ES0 is already running this has no effect until stop & start is called.
 * @param enabled If true, HPA mode is enabled, LPA mode otherwise.
 */
void es0_enable_hpa_mode(bool enabled);

/**
 * @brief Register a user of a ES0
 * @param baudrate Baudrate used in host side will be passed to LL. All instances must
 *     use same so once set it can only be changed by stopping all instances
 *     first and then reinitialize with new value
 * @retval  0 If successful
 * @retval  -1 If too many users
 * @retval  -2 If calculated size of boot params > 512
 * @retval  -3 If calculated size of boot params differs from actual size.
 * @retval  -4 Starting ES0 failed
 * @retval  -5 Baudrate has not been set for HCI/AHI UARTs
 * @retval  -6 Baudrate mismatch
 */
int8_t take_es0_into_use(void);

/**
 * @brief Register a user of a ES0 with dynamic parameters
 * @param nvds_buff NVDS configuration defined by user
 * @param nvds_size Length of nvds data
 * @param clock_select ES0 clock select
 * @param hpa_mode If true, HPA mode is used, otherwise LPA mode is used
 *
 * @retval  0 If successful
 * @retval  -1 If too many users
 * @retval  -4 Starting ES0 failed
 */
int8_t take_es0_into_use_with_params(uint8_t *nvds_buff, uint16_t nvds_size, uint32_t clock_select,
				     bool hpa_mode);

/**
 * @brief De-register a user of a ES0
 * @retval  -1 If no active users
 * @retval  -2 Shutdown of ES0 failed
 */
int8_t stop_using_es0(void);

/**
 * @brief wakeup ES0 using uart
 *
 * ES0 needs to be woken once per boot and should then remain active
 * until ES1 is powered off.
 *
 * This function can be called as many times during the boot.
 */
void wake_es0(const struct device *uart_dev);

#endif /* __ES0_POWER_MANAGER_H__ */
