/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

#ifndef __NSS_DP_ARCH_H__
#define __NSS_DP_ARCH_H__

#define NSS_DP_HAL_MAX_PORTS		6
#define NSS_DP_HAL_CPU_NUM		4
#define NSS_DP_HAL_START_IFNUM		1

/*
 * Number of TX/RX queue supported
 */
#define NSS_DP_QUEUE_NUM		4

/*
 * TX/RX NAPI budget
 */
#define NSS_DP_HAL_RX_NAPI_BUDGET	128
#define NSS_DP_HAL_TX_NAPI_BUDGET	128

/**
 * nss_dp_hal_gmac_stats
 *	The per-GMAC statistics structure.
 */
struct nss_dp_hal_gmac_stats {
	uint64_t rx_packets;		/**< Number of RX packets */
	uint64_t rx_bytes;		/**< Number of RX bytes */
	uint64_t rx_dropped;		/**< Number of RX dropped packets */
	uint64_t rx_fraglist_packets;	/**< Number of RX fraglist packets */
	uint64_t rx_nr_frag_packets;	/**< Number of RX nr fragment packets */
	uint64_t rx_nr_frag_headroom_err;
			/**< Number of RX nr fragment packets with headroom error */
	uint64_t tx_packets;		/**< Number of TX packets */
	uint64_t tx_bytes;		/**< Number of TX bytes */
	uint64_t tx_dropped;		/**< Number of TX dropped packets */
	uint64_t tx_nr_frag_packets;	/**< Number of TX nr fragment packets */
	uint64_t tx_fraglist_packets;	/**< Number of TX fraglist packets */
	uint64_t tx_fraglist_with_nr_frags_packets;	/**< Number of TX fraglist packets with nr fragments */
	uint64_t tx_tso_packets;	/**< Number of TX TCP segmentation offload packets */
};

extern int edma_init(void);
extern void edma_cleanup(bool is_dp_override);
extern struct nss_dp_data_plane_ops nss_dp_edma_ops;

#endif /* __NSS_DP_ARCH_H__ */
