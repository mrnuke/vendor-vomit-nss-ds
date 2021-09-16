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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/debugfs.h>

#include <nss_dp_dev.h>
#include "syn_dma_reg.h"

/*
 * syn_dp_rx_reset_qptr()
 *	Reset the descriptor after Rx is over.
 */
static inline void syn_dp_rx_reset_qptr(struct syn_dp_info_rx *rx_info)
{
	uint32_t rxnext = rx_info->rx_idx;
	struct dma_desc_rx *rxdesc = rx_info->rx_desc + rxnext;

#ifdef SYN_DP_DEBUG
	BUG_ON(syn_dp_gmac_is_rx_desc_owned_by_dma(rx_info->rx_desc + rxnext));
#endif
	rx_info->rx_idx = (rxnext + 1) & (SYN_DP_RX_DESC_SIZE - 1);

	rx_info->rx_buf_pool[rxnext].skb = NULL;
	rxdesc->status = 0;
	rxdesc->length &= DESC_RX_DESC_END_OF_RING;
	rxdesc->buffer1 = 0;
	rxdesc->buffer2 = 0;
	rxdesc->reserved1 = 0;

	/*
	 * This returns one descriptor to processor. So busy count will be decremented by one.
	 */
	atomic_dec((atomic_t *)&rx_info->busy_rx_desc_cnt);
}

/*
 * syn_dp_rx_set_qptr()
 *	Prepares the descriptor to receive packets.
 */
static inline int32_t syn_dp_rx_set_qptr(struct syn_dp_info_rx *rx_info,
					uint32_t Buffer1, uint32_t Length1, struct sk_buff *skb)
{
	uint32_t rxnext = rx_info->rx_refill_idx;
	struct dma_desc_rx *rxdesc = rx_info->rx_desc + rxnext;

#ifdef SYN_DP_DEBUG
	BUG_ON(atomic_read((atomic_t *)&rx_info->busy_rx_desc_cnt) >= SYN_DP_RX_DESC_SIZE);
	BUG_ON(rxdesc != (rx_info->rx_desc + rxnext));
	BUG_ON(!syn_dp_gmac_is_rx_desc_empty(rxdesc));
	BUG_ON(syn_dp_gmac_is_rx_desc_owned_by_dma(rxdesc));
#endif

	if (Length1 > SYN_DP_MAX_DESC_BUFF) {
		rxdesc->length |= (SYN_DP_MAX_DESC_BUFF << DESC_SIZE1_SHIFT) & DESC_SIZE1_MASK;
		rxdesc->length |= ((Length1 - SYN_DP_MAX_DESC_BUFF) << DESC_SIZE2_SHIFT) & DESC_SIZE2_MASK;
	} else {
		rxdesc->length |= ((Length1 << DESC_SIZE1_SHIFT) & DESC_SIZE1_MASK);
	}

	rxdesc->buffer1 = Buffer1;
	rx_info->rx_buf_pool[rxnext].skb = skb;

	/*
	 * Program second buffer address if using two buffers.
	 */
	if (Length1 > SYN_DP_MAX_DESC_BUFF)
		rxdesc->buffer2 = Buffer1 + SYN_DP_MAX_DESC_BUFF;
	else
		rxdesc->buffer2 = 0;

	rxdesc->extstatus = 0;
	rxdesc->timestamplow = 0;
	rxdesc->timestamphigh = 0;

	/*
	 * Ensure all write completed before setting own by dma bit so when gmac
	 * HW takeover this descriptor, all the fields are filled correctly
	 */
	wmb();
	rxdesc->status = DESC_OWN_BY_DMA;

	rx_info->rx_refill_idx = (rxnext + 1) & (SYN_DP_RX_DESC_SIZE - 1);

	/*
	 * 1 descriptor will be given to HW. So busy count incremented by 1.
	 */
	atomic_inc((atomic_t *)&rx_info->busy_rx_desc_cnt);

	return rxnext;
}

/*
 * syn_dp_rx_refill()
 *	Refill the RX descrptor
 */
void syn_dp_rx_refill(struct syn_dp_info_rx *rx_info)
{
	struct net_device *netdev = rx_info->netdev;
	int empty_count = SYN_DP_RX_DESC_SIZE - atomic_read((atomic_t *)&rx_info->busy_rx_desc_cnt);
	int i;

	dma_addr_t dma_addr;
	struct sk_buff *skb;

	for (i = 0; i < empty_count; i++) {
		skb = __netdev_alloc_skb(netdev, SYN_DP_MINI_JUMBO_FRAME_MTU, GFP_ATOMIC);
		if (unlikely(skb == NULL)) {
			netdev_dbg(netdev, "Unable to allocate skb, will try next time\n");
			break;
		}

		skb_reserve(skb, NET_IP_ALIGN);

		dma_addr = dma_map_single(rx_info->dev, skb->data, SYN_DP_MINI_JUMBO_FRAME_MTU, DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(rx_info->dev, dma_addr))) {
			dev_kfree_skb(skb);
			netdev_dbg(netdev, "DMA mapping failed for empty buffer\n");
			break;
		}

		syn_dp_rx_set_qptr(rx_info, dma_addr, SYN_DP_MINI_JUMBO_FRAME_MTU, skb);
	}

	syn_resume_dma_rx(rx_info->mac_base);
}

/*
 * syn_dp_rx()
 *	Process RX packets
 */
int syn_dp_rx(struct syn_dp_info_rx *rx_info, int budget)
{
	struct dma_desc_rx *desc = NULL;
	int frame_length, busy;
	uint32_t status, extstatus;
	struct sk_buff *rx_skb;
	struct net_device *netdev = rx_info->netdev;
	void __iomem *mac_base = rx_info->mac_base;
	uint32_t rx_packets = 0, rx_bytes = 0;

	atomic64_add(syn_get_rx_missed(mac_base), (atomic64_t *)&rx_info->rx_stats.rx_missed);
	atomic64_add(syn_get_fifo_overflows(mac_base), (atomic64_t *)&rx_info->rx_stats.rx_fifo_overflows);

	busy = atomic_read((atomic_t *)&rx_info->busy_rx_desc_cnt);
	if (!busy) {
		/*
		 * No desc are held by gmac dma, we are done
		 */
		return 0;
	}

	if (busy > budget) {
		busy = budget;
	}

	do {
		desc = rx_info->rx_desc + rx_info->rx_idx;
		if (syn_dp_gmac_is_rx_desc_owned_by_dma(desc)) {
			/*
			 * Desc still hold by gmac dma, so we are done
			 */
			break;
		}

		status = desc->status;

		rx_skb = rx_info->rx_buf_pool[rx_info->rx_idx].skb;

		dma_unmap_single(rx_info->dev, desc->buffer1,
				SYN_DP_MINI_JUMBO_FRAME_MTU, DMA_FROM_DEVICE);

		extstatus = desc->extstatus;
		if (likely(syn_dp_gmac_is_rx_desc_valid(status) && !(extstatus & (DESC_RX_IP_HEADER_ERROR | DESC_RX_IP_PAYLOAD_ERROR)))) {
			/*
			 * We have a pkt to process get the frame length
			 */
			frame_length = syn_dp_gmac_get_rx_desc_frame_length(status);

			/*
			 * Get rid of FCS: 4
			 */
			frame_length -= ETH_FCS_LEN;

			/*
			 * Valid packet, collect stats
			 */
			rx_packets++;
			rx_bytes += frame_length;

			/*
			 * Type_trans and deliver to linux
			 */
			skb_put(rx_skb, frame_length);
			rx_skb->protocol = eth_type_trans(rx_skb, netdev);
			if (likely(!(extstatus & DESC_RX_CHK_SUM_BYPASS))) {
				rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
			} else {
				skb_checksum_none_assert(rx_skb);
			}

			netif_receive_skb(rx_skb);
		} else {
			atomic64_inc((atomic64_t *)&rx_info->rx_stats.rx_errors);
			dev_kfree_skb_any(rx_skb);

			if (status & (DESC_RX_CRC | DESC_RX_COLLISION |
					DESC_RX_OVERFLOW | DESC_RX_DRIBBLING |
					DESC_RX_LENGTH_ERROR)) {
				(status & DESC_RX_COLLISION) ? atomic64_inc((atomic64_t *)&rx_info->rx_stats.rx_late_collision_errors) : NULL;
				(status & DESC_RX_DRIBBLING) ? atomic64_inc((atomic64_t *)&rx_info->rx_stats.rx_dribble_bit_errors) : NULL;
				(status & DESC_RX_LENGTH_ERROR) ? atomic64_inc((atomic64_t *)&rx_info->rx_stats.rx_length_errors) : NULL;
				(status & DESC_RX_CRC) ? atomic64_inc((atomic64_t *)&rx_info->rx_stats.rx_crc_errors) : NULL;
				(status & DESC_RX_OVERFLOW) ? atomic64_inc((atomic64_t *)&rx_info->rx_stats.rx_overflow_errors) : NULL;
			}
		}

		syn_dp_rx_reset_qptr(rx_info);
		busy--;
	} while (likely(busy > 0));

	/*
	 * Increment total rx packets and byte count.
	 */
	atomic64_add(rx_packets, (atomic64_t *)&rx_info->rx_stats.rx_packets);
	atomic64_add(rx_bytes, (atomic64_t *)&rx_info->rx_stats.rx_bytes);

	return budget - busy;
}
