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

#include <asm/cacheflush.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/debugfs.h>
#include <nss_dp_dev.h>
#include "syn_dma_reg.h"

/*
 * syn_dp_tx_reset_qptr
 *	Reset the descriptor after Tx is over.
 */
static inline void syn_dp_tx_reset_qptr(struct syn_dp_info_tx *tx_info)
{
	struct dma_desc_tx *txdesc;
	uint32_t txover = tx_info->tx_comp_idx;

	txdesc = tx_info->tx_desc + tx_info->tx_comp_idx;
	txdesc->status &= DESC_TX_DESC_END_OF_RING;
	txdesc->length = 0;
	txdesc->buffer1 = 0;
	txdesc->buffer2 = 0;
	txdesc->reserved1 = 0;

	tx_info->tx_comp_idx = (txover + 1) & (SYN_DP_TX_DESC_SIZE - 1);
	tx_info->tx_buf_pool[txover].skb = NULL;

	/*
	 * Busy tx descriptor is reduced by one as
	 * it will be handed over to Processor now.
	 */
	atomic_dec((atomic_t *)&tx_info->busy_tx_desc_cnt);
}

/*
 * syn_dp_tx_set_qptr
 *	Populate the tx desc structure with the buffer address.
 */
static inline struct dma_desc_tx *syn_dp_tx_set_qptr(struct syn_dp_info_tx *tx_info,
					   uint32_t buffer1, uint32_t length1, struct sk_buff *skb, uint32_t offload_needed,
					   uint32_t tx_cntl, uint32_t set_dma)
{
	uint32_t txnext = tx_info->tx_idx;
	struct dma_desc_tx *txdesc = tx_info->tx_desc + txnext;
	uint32_t tx_skb_index = txnext;

#ifdef SYN_DP_DEBUG
	BUG_ON(atomic_read((atomic_t *)&tx_info->busy_tx_desc_cnt) > SYN_DP_TX_DESC_SIZE);
	BUG_ON(txdesc != (tx_info->tx_desc + txnext));
	BUG_ON(!syn_dp_gmac_is_tx_desc_empty(txdesc));
	BUG_ON(syn_dp_gmac_is_tx_desc_owned_by_dma(txdesc));
#endif

	if (length1 > SYN_DP_MAX_DESC_BUFF_LEN) {
		txdesc->length = (SYN_DP_MAX_DESC_BUFF_LEN << DESC_SIZE1_SHIFT) & DESC_SIZE1_MASK;
		txdesc->length |=
		    ((length1 - SYN_DP_MAX_DESC_BUFF_LEN) << DESC_SIZE2_SHIFT) & DESC_SIZE2_MASK;
	} else {
		txdesc->length = ((length1 << DESC_SIZE1_SHIFT) & DESC_SIZE1_MASK);
	}

	txdesc->status |= tx_cntl;
	txdesc->buffer1 = buffer1;

	tx_info->tx_buf_pool[tx_skb_index].skb = skb;

	/* Program second buffer address if using two buffers. */
	if (length1 > SYN_DP_MAX_DESC_BUFF_LEN)
		txdesc->buffer2 = buffer1 + SYN_DP_MAX_DESC_BUFF_LEN;
	else
		txdesc->buffer2 = 0;

	if (likely(offload_needed)) {
		txdesc->status |= DESC_TX_CIS_TCP_PSEUDO_CS;
	}

	/*
	 * Ensure all write completed before setting own by dma bit so when gmac
	 * HW takeover this descriptor, all the fields are filled correctly
	 */
	wmb();
	txdesc->status |= set_dma;

	tx_info->tx_idx = (txnext + 1) & (SYN_DP_TX_DESC_SIZE - 1);
	return txdesc;
}

/*
 * syn_dp_tx_queue_desc
 *	Queue TX descriptor to the TX ring
 */
static void syn_dp_tx_desc_queue(struct syn_dp_info_tx *tx_info, struct sk_buff *skb, dma_addr_t dma_addr)
{
	unsigned int len = skb->len;

	syn_dp_tx_set_qptr(tx_info, dma_addr, len, skb, (skb->ip_summed == CHECKSUM_PARTIAL),
				(DESC_TX_LAST | DESC_TX_FIRST | DESC_TX_INT_ENABLE), DESC_OWN_BY_DMA);
}

/*
 * syn_dp_tx_complete
 *	Xmit complete, clear descriptor and free the skb
 */
int syn_dp_tx_complete(struct syn_dp_info_tx *tx_info, int budget)
{
	int busy, len;
	uint32_t status;
	struct dma_desc_tx *desc = NULL;
	struct sk_buff *skb;
	uint32_t tx_skb_index;

	busy = atomic_read((atomic_t *)&tx_info->busy_tx_desc_cnt);

	if (!busy) {
		/* No desc are hold by gmac dma, we are done */
		return 0;
	}

	if (busy > budget)
		busy = budget;

	do {
		desc = tx_info->tx_desc + tx_info->tx_comp_idx;
		if (syn_dp_gmac_is_tx_desc_owned_by_dma(desc)) {
			/* desc still hold by gmac dma, so we are done */
			break;
		}

		len = (desc->length & DESC_SIZE1_MASK) >> DESC_SIZE1_SHIFT;
		dma_unmap_single(tx_info->dev, desc->buffer1, len, DMA_TO_DEVICE);

		status = desc->status;
		if (status & DESC_TX_LAST) {
			/* TX is done for this whole skb, we can free it */
			/* Get the skb from the tx skb pool */
			tx_skb_index = tx_info->tx_comp_idx;
			skb = tx_info->tx_buf_pool[tx_skb_index].skb;

#ifdef SYN_DP_DEBUG
			BUG_ON(!skb);
#endif
			dev_kfree_skb_any(skb);

			if (unlikely(status & DESC_TX_ERROR)) {
				/* Some error happen, collect statistics */
			} else {
				/* No error, recored tx pkts/bytes and
				 * collision
				 */
			}
		}
		syn_dp_tx_reset_qptr(tx_info);
		busy--;
	} while (likely(busy > 0));

	return budget - busy;
}

/*
 * syn_dp_tx
 *	TX routine for Synopsys GMAC
 */
int syn_dp_tx(struct syn_dp_info_tx *tx_info, struct sk_buff *skb)
{
	struct net_device *netdev = tx_info->netdev;
	unsigned len = skb->len;
	dma_addr_t dma_addr;

	/*
	 * If we don't have enough tx descriptor for this pkt, return busy.
	 */
	if ((SYN_DP_TX_DESC_SIZE - atomic_read((atomic_t *)&tx_info->busy_tx_desc_cnt)) < 1) {
		netdev_dbg(netdev, "Not enough descriptors available");
		return -1;
	}

	dma_addr = dma_map_single(tx_info->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(tx_info->dev, dma_addr))) {
		netdev_dbg(netdev, "DMA mapping failed for empty buffer\n");
		return -1;
	}

	/*
	 * Queue packet to the GMAC rings
	 */
	syn_dp_tx_desc_queue(tx_info, skb, dma_addr);

	syn_resume_dma_tx(tx_info->mac_base);
	atomic_inc((atomic_t *)&tx_info->busy_tx_desc_cnt);

	return 0;
}
