/*
 * Copyright (c) 2025 Alifsemiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/drivers/hwinfo.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>


ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
/* need to implement properly  */
	memcpy(buffer, "1200", length);

	return length;
}


