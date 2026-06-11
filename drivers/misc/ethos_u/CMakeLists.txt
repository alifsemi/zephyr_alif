# Copyright 2022 Arm Limited and/or its affiliates <open-source-office@arm.com>
#
# SPDX-License-Identifier: Apache-2.0

# Compile driver only if there are enabled Ethos-U devices in the device tree
# Zephyr auto-generates CONFIG_DT_HAS_ARM_ETHOS_U_ENABLED when device tree has
# a node with compatible="arm,ethos-u" and status="okay"
if(CONFIG_DT_HAS_ARM_ETHOS_U_ENABLED)
  zephyr_library()
  zephyr_library_sources(ethos_u.c)
  # Expose the Ethos-U core driver's private headers (src/) to this library
  # only, so device-layer APIs such as ethosu_dev_set_clock_and_power() can be
  # used for power management.
  zephyr_library_include_directories(${ZEPHYR_HAL_ETHOS_U_MODULE_DIR}/src)
endif()
