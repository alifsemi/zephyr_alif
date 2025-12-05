# Copyright 2022 Arm Limited and/or its affiliates <open-source-office@arm.com>
#
# SPDX-License-Identifier: Apache-2.0

# Compile driver only if there are enabled Ethos-U devices in the device tree
# Zephyr auto-generates CONFIG_DT_HAS_ARM_ETHOS_U_ENABLED when device tree has
# a node with compatible="arm,ethos-u" and status="okay"
if(CONFIG_DT_HAS_ARM_ETHOS_U_ENABLED)
  zephyr_library()
  zephyr_library_sources(ethos_u.c)
endif()
