/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ns), okay)
GROUP_START(NON_SECURE0)
	SECTION_PROLOGUE(_ALIF_NS_SECTION_NAME, (NOLOAD),)
	{
			KEEP(*(.alif_ns))
			KEEP(*(".alif_ns.*"))
			KEEP(*(.bss.usb_dwc3_event_buff))
			KEEP(*(.bss.udc_dwc3_priv_0))
			KEEP(*(.noinit.*kheap_buf_video_buffer_pool))
	} GROUP_LINK_IN(NON_SECURE0)

GROUP_END(NON_SECURE0)
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(ns), okay) */
