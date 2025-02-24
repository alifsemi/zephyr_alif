# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})
zephyr_include_directories(common)
zephyr_sources(common/soc_ensemble_common.S)

zephyr_sources(common/soc_common.c)
