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
			KEEP(*(.bss.udc_dwc3_event_buff))
			KEEP(*(.bss.udc_dwc3_priv_0))
			KEEP(*(.noinit.*kheap_buf_video_buffer_pool))
			KEEP(*(.bss.net_buf_data_cdc_acm_ep_pool))
			KEEP(*(.noinit.*kheap_buf_net_buf_mem_pool_udc_ep_pool))
			KEEP(*net_pkt.c.obj*(.noinit*))
			KEEP(*resolve.c.obj*(.noinit*))
			KEEP(*dispatcher.c.obj*(.noinit*))
	} GROUP_LINK_IN(NON_SECURE0)

GROUP_END(NON_SECURE0)
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(ns), okay) */
