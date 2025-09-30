# Copyright (c) 2024 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

#ifdef CONFIG_SOC_B1_FPGA_RTSS_HE
zephyr_sources_ifdef(CONFIG_SOC_B1_FPGA_RTSS_HE soc_b1_fpga_rtss_he.c)
#else
zephyr_sources_ifdef(CONFIG_SOC_B1_DK_RTSS_HE soc_b1_dk_rtss_he.c)
#endif

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/arm/alif_balletto/common/linker.ld CACHE INTERNAL "")
