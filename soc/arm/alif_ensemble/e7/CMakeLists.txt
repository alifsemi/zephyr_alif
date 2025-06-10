# Copyright (c) 2024 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0
zephyr_sources_ifdef(CONFIG_SOC_E7_DK_RTSS_HE soc_e7_dk_rtss_he.c)
zephyr_sources_ifdef(CONFIG_SOC_E7_DK_RTSS_HP soc_e7_dk_rtss_hp.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/arm/alif_ensemble/common/linker.ld CACHE INTERNAL "")
