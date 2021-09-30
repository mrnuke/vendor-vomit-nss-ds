/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __NSS_DP_SYN_DP_RX__
#define __NSS_DP_SYN_DP_RX__

#define SYN_DP_NAPI_BUDGET_RX		32
#define SYN_DP_RX_DESC_SIZE		128	/* Rx Descriptors needed in the descriptor pool/queue */
#define SYN_DP_RX_DESC_MAX_INDEX	(SYN_DP_RX_DESC_SIZE - 1)

/*
 * syn_dp_rx_buf
 */
struct syn_dp_rx_buf {
	struct sk_buff *skb;	/* Buffer pointer populated to Rx/Tx dma desc */
	size_t map_addr_virt;	/* Virtual address of buffer populated to Rx/Tx dma desc */
};

/*
 * syn_dp_info_rx
 */
struct syn_dp_info_rx {
	struct napi_struct napi_rx;	/* Rx NAPI */
	void __iomem *mac_base;		/* MAC base for register read/write */
	struct dma_desc_rx *rx_desc;	/* start address of RX descriptors ring or
					   chain, this is used by the driver */
	uint32_t busy_rx_desc_cnt;	/* Number of Rx Descriptors owned by
					   DMA at any given time */
	uint32_t rx_refill_idx;		/* index of the rx descriptor owned by DMA */
	uint32_t rx_idx;		/* index of the rx descriptor next available with driver */
	struct syn_dp_rx_buf rx_buf_pool[SYN_DP_RX_DESC_SIZE];
					/* Rx skb pool helping RX DMA descriptors */
	struct nss_dp_hal_gmac_stats_rx rx_stats;
					/* GMAC driver Rx statistics */
	struct net_device *netdev;	/* Net-device corresponding to the GMAC */
	struct device *dev;		/* Platform device corresponding to the GMAC */
};

#endif /*  __NSS_DP_SYN_DP_RX__ */
