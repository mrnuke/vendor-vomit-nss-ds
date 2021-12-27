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
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
 * RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
 * USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/reset.h>
#include <fal/fal_qos.h>
#include "edma.h"
#include "edma_cfg_rx.h"
#include "edma_regs.h"
#include "edma_debug.h"
#include "nss_dp_dev.h"

/*
 * edma_cfg_rx_fill_ring_cleanup()
 *	Cleanup resources for one RxFill ring
 *
 * API expects ring to be disabled by caller
 */
static void edma_cfg_rx_fill_ring_cleanup(struct edma_gbl_ctx *egc,
				struct edma_rxfill_ring *rxfill_ring)
{
	uint16_t cons_idx, curr_idx;
	uint32_t reg_data;

	/*
	 * Get RXFILL ring producer index
	 */
	curr_idx = rxfill_ring->prod_idx & EDMA_RXFILL_PROD_IDX_MASK;

	/*
	 * Get RXFILL ring consumer index
	 */
	reg_data = edma_reg_read(EDMA_REG_RXFILL_CONS_IDX(rxfill_ring->ring_id));
	cons_idx = reg_data & EDMA_RXFILL_CONS_IDX_MASK;

	while (curr_idx != cons_idx) {
		struct sk_buff *skb;
		struct edma_rxfill_desc *rxfill_desc;

		/*
		 * Get RXFILL descriptor
		 */
		rxfill_desc = EDMA_RXFILL_DESC(rxfill_ring, cons_idx);

		cons_idx = (cons_idx + 1) & EDMA_RX_RING_SIZE_MASK;

		/*
		 * Get skb from opaque
		 */
		skb = (struct sk_buff *)EDMA_RXFILL_OPAQUE_GET(rxfill_desc);
		if (unlikely(!skb)) {
			edma_warn("Empty skb reference at index:%d\n",
					cons_idx);
			continue;
		}
		dev_kfree_skb_any(skb);
	}

	/*
	 * Free RXFILL ring descriptors
	 */
	kfree(rxfill_ring->desc);
	rxfill_ring->desc = NULL;
	rxfill_ring->dma = (dma_addr_t)0;
}

/*
 * edma_cfg_rx_fill_ring_setup()
 *	Setup resources for one RxFill ring
 */
static int edma_cfg_rx_fill_ring_setup(struct edma_rxfill_ring *rxfill_ring)
{
	/*
	 * Allocate RxFill ring descriptors
	 */
	rxfill_ring->desc = kmalloc((sizeof(struct edma_rxfill_desc) * rxfill_ring->count) +
				SMP_CACHE_BYTES,  GFP_KERNEL | __GFP_ZERO);
	if (!rxfill_ring->desc) {
		edma_err("Descriptor alloc for RXFILL ring %u failed\n",
							rxfill_ring->ring_id);
		return -ENOMEM;
	}

	rxfill_ring->dma = (dma_addr_t)virt_to_phys(rxfill_ring->desc);

	return 0;
}

/*
 * edma_cfg_rx_desc_ring_setup()
 *	Setup resources for one RxDesc ring
 */
static int edma_cfg_rx_desc_ring_setup(struct edma_rxdesc_ring *rxdesc_ring)
{
	/*
	 * Allocate RxDesc ring descriptors
	 */
	rxdesc_ring->pdesc = kmalloc((sizeof(struct edma_rxdesc_desc) *  rxdesc_ring->count) +
				SMP_CACHE_BYTES,  GFP_KERNEL | __GFP_ZERO);
	if (!rxdesc_ring->pdesc) {
		edma_err("Descriptor alloc for RXDESC ring %u failed\n",
							rxdesc_ring->ring_id);
		return -ENOMEM;
	}

	rxdesc_ring->pdma = (dma_addr_t)virt_to_phys(rxdesc_ring->pdesc);

	/*
	 * Allocate secondary RxDesc ring descriptors
	 */
	rxdesc_ring->sdesc = kmalloc((sizeof(struct edma_rxdesc_sec_desc) *  rxdesc_ring->count) +
				SMP_CACHE_BYTES,  GFP_KERNEL | __GFP_ZERO);
	if (!rxdesc_ring->sdesc) {
		edma_err("Descriptor alloc for secondary RX ring %u failed\n",
							rxdesc_ring->ring_id);
		kfree(rxdesc_ring->pdesc);
		rxdesc_ring->pdesc = NULL;
		rxdesc_ring->pdma = (dma_addr_t)0;
		return -ENOMEM;
	}

	rxdesc_ring->sdma = (dma_addr_t)virt_to_phys(rxdesc_ring->sdesc);

	return 0;
}

/*
 * edma_cfg_rx_desc_ring_cleanup()
 *	Cleanup resources for RxDesc ring
 *
 * API expects ring to be disabled by caller
 */
static void edma_cfg_rx_desc_ring_cleanup(struct edma_gbl_ctx *egc,
				struct edma_rxdesc_ring *rxdesc_ring)
{
	uint16_t prod_idx, cons_idx;

	/*
	 * Get Rxdesc consumer & producer indices
	 */
	cons_idx = rxdesc_ring->cons_idx & EDMA_RXDESC_CONS_IDX_MASK;

	prod_idx = edma_reg_read(EDMA_REG_RXDESC_PROD_IDX(rxdesc_ring->ring_id))
					& EDMA_RXDESC_PROD_IDX_MASK;

	/*
	 * Free any buffers assigned to any descriptors
	 */
	while (cons_idx != prod_idx) {
		struct sk_buff *skb;
		struct edma_rxdesc_desc *rxdesc_desc =
			EDMA_RXDESC_PRI_DESC(rxdesc_ring, cons_idx);

		/*
		 * Update consumer index
		 */
		cons_idx = (cons_idx + 1) & EDMA_RX_RING_SIZE_MASK;

		/*
		 * Get opaque from RXDESC
		 */
		skb = (struct sk_buff *)EDMA_RXDESC_OPAQUE_GET(rxdesc_desc);
		if (unlikely(!skb)) {
			edma_warn("Empty skb reference at index:%d\n",
								cons_idx);
			continue;
		}
		dev_kfree_skb_any(skb);
	}

	/*
	 * Free RXDESC ring descriptors
	 */
	kfree(rxdesc_ring->pdesc);
	rxdesc_ring->pdesc = NULL;
	rxdesc_ring->pdma = (dma_addr_t)0;

	/*
	 * TODO:
	 * Free any buffers assigned to any secondary ring descriptors
	 */
	kfree(rxdesc_ring->sdesc);
	rxdesc_ring->sdesc = NULL;
	rxdesc_ring->sdma = (dma_addr_t)0;
}

/*
 * edma_cfg_rx_desc_rings_reset_queue_mapping()
 *	API to reset Rx descriptor rings to PPE queues mapping
 */
static int32_t edma_cfg_rx_desc_rings_reset_queue_mapping(void)
{
	fal_queue_bmp_t queue_bmp = {0};
	int32_t i;

	for (i = 0; i < EDMA_MAX_RXDESC_RINGS; i++) {
		if (fal_edma_ring_queue_map_set(EDMA_SWITCH_DEV_ID, i, &queue_bmp) != SW_OK) {
			edma_err("Error in unmapping rxdesc ring %d to PPE queue mapping to"
					" disable its backpressure configuration\n", i);
			return -1;
		}
	}

	return 0;
}

/*
 * edma_cfg_rx_desc_ring_reset_queue_priority()
 *	API to reset the priority for PPE queues mapped to Rx rings
 */
static int32_t edma_cfg_rx_desc_ring_reset_queue_priority(struct edma_gbl_ctx *egc,
				uint32_t rxdesc_ring_idx)
{
	fal_qos_scheduler_cfg_t qsch;
	uint32_t i, queue_id, bit_set, port_id, cur_queue_word;

	for (i = 0; i < EDMA_RING_MAPPED_QUEUE_BM_WORD_COUNT; i++) {
		cur_queue_word = egc->rxdesc_ring_to_queue_bm[rxdesc_ring_idx][i];
		if (cur_queue_word == 0) {
			continue;
		}

		do {
			bit_set = ffs((uint32_t)cur_queue_word);
			queue_id = (((i * EDMA_BITS_IN_WORD) + bit_set) - 1);
			memset(&qsch, 0, sizeof(fal_qos_scheduler_cfg_t));
			if (fal_queue_scheduler_get(EDMA_SWITCH_DEV_ID, queue_id,
						EDMA_PPE_QUEUE_LEVEL, &port_id, &qsch) != SW_OK) {
				edma_err("Error in getting %u queue's priority information\n", queue_id);
				return -1;
			}

			/*
			 * Configure the default queue priority.
			 * In IPQ95xx, currently single queue is being mapped to the
			 * single Rx descriptor ring and each ring will be processed
			 * on the separate core. Therefore assigning same priority to
			 * all these mapped queues.
			 */
			qsch.e_pri = EDMA_RX_DEFAULT_QUEUE_PRI;
			qsch.c_pri = EDMA_RX_DEFAULT_QUEUE_PRI;
			if (fal_queue_scheduler_set(EDMA_SWITCH_DEV_ID, queue_id,
						EDMA_PPE_QUEUE_LEVEL, port_id, &qsch) != SW_OK) {
				edma_err("Error in resetting %u queue's priority\n", queue_id);
				return -1;
			}

			cur_queue_word &= ~(1 << (bit_set - 1));
		} while (cur_queue_word);
	}

	return 0;
}

/*
 * edma_cfg_rx_desc_ring_reset_queue_config()
 *	API to reset the Rx descriptor rings configurations
 */
static int32_t edma_cfg_rx_desc_ring_reset_queue_config(struct edma_gbl_ctx *egc)
{
	int32_t i;

	/*
	 * Unmap Rxdesc ring to PPE queue mapping to reset its backpressure configuration
	 */
	if (edma_cfg_rx_desc_rings_reset_queue_mapping()) {
		edma_err("Error in resetting Rx desc ring backpressure configurations\n");
		return -1;
	}

	/*
	 * Reset the priority for PPE queues mapped to Rx rings
	 */
	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		if(edma_cfg_rx_desc_ring_reset_queue_priority(egc, i)) {
			edma_err("Error in resetting ring:%d queue's priority\n",
					 i + egc->rxdesc_ring_start);
			return -1;
		}
	}

	return 0;
}

/*
 * edma_cfg_rx_desc_ring_configure()
 *	Configure one RxDesc ring in EDMA HW
 */
static void edma_cfg_rx_desc_ring_configure(struct edma_gbl_ctx *egc,
					struct edma_rxdesc_ring *rxdesc_ring)
{
	uint32_t data;

	edma_reg_write(EDMA_REG_RXDESC_BA(rxdesc_ring->ring_id),
			(uint32_t)(rxdesc_ring->pdma & EDMA_RXDESC_BA_MASK));

	edma_reg_write(EDMA_REG_RXDESC_PREHEADER_BA(rxdesc_ring->ring_id),
			(uint32_t)(rxdesc_ring->sdma & EDMA_RXDESC_PREHEADER_BA_MASK));

	data = rxdesc_ring->count & EDMA_RXDESC_RING_SIZE_MASK;
	data |= (EDMA_RXDESC_PL_DEFAULT_VALUE & EDMA_RXDESC_PL_OFFSET_MASK)
		 << EDMA_RXDESC_PL_OFFSET_SHIFT;
	edma_reg_write(EDMA_REG_RXDESC_RING_SIZE(rxdesc_ring->ring_id), data);

	/*
	 * Configure the default timer mitigation value
	 */
	data = (EDMA_RX_MOD_TIMER_INIT & EDMA_RX_MOD_TIMER_INIT_MASK)
			<< EDMA_RX_MOD_TIMER_INIT_SHIFT;
	edma_reg_write(EDMA_REG_RX_MOD_TIMER(rxdesc_ring->ring_id), data);

	/*
	 * Enable ring. Set ret mode to 'opaque'.
	 */
	edma_reg_write(EDMA_REG_RX_INT_CTRL(rxdesc_ring->ring_id), EDMA_RX_NE_INT_EN);
}

/*
 * edma_cfg_rx_fill_ring_configure()
 *	Configure one RxFill ring in EDMA HW
 */
static void edma_cfg_rx_fill_ring_configure(struct edma_gbl_ctx *egc,
					struct edma_rxfill_ring *rxfill_ring)
{
	uint32_t ring_sz;

	edma_reg_write(EDMA_REG_RXFILL_BA(rxfill_ring->ring_id),
			(uint32_t)(rxfill_ring->dma & EDMA_RING_DMA_MASK));

	ring_sz = rxfill_ring->count & EDMA_RXFILL_RING_SIZE_MASK;
	edma_reg_write(EDMA_REG_RXFILL_RING_SIZE(rxfill_ring->ring_id), ring_sz);

	/*
	 * Alloc Rx buffers
	 */
	edma_rx_alloc_buffer(rxfill_ring, rxfill_ring->count - 1);
}

/*
 * edma_cfg_rx_qid_to_rx_desc_ring_mapping()
 *	Configure PPE queue id to Rx ring mapping
 */
static void edma_cfg_rx_qid_to_rx_desc_ring_mapping(struct edma_gbl_ctx *egc)
{
	uint32_t desc_index, i;
	uint32_t reg_index, data;

	/*
	 * Set PPE QID to EDMA Rx ring mapping.
	 * Each entry can hold mapping for 4 PPE queues and
	 * entry size is 4 bytes.
	 */
	desc_index = (egc->rxdesc_ring_start & EDMA_RX_RING_ID_MASK);

	for (i = EDMA_PORT_QUEUE_START;
		i <= EDMA_PORT_QUEUE_END;
			i += EDMA_QID2RID_NUM_PER_REG) {
		reg_index = i/EDMA_QID2RID_NUM_PER_REG;
		data = EDMA_RX_RING_ID_QUEUE0_SET(desc_index) |
			EDMA_RX_RING_ID_QUEUE1_SET(desc_index + 1) |
			EDMA_RX_RING_ID_QUEUE2_SET(desc_index + 2) |
			EDMA_RX_RING_ID_QUEUE3_SET(desc_index + 3);

		edma_reg_write(EDMA_QID2RID_TABLE_MEM(reg_index), data);
		desc_index += EDMA_QID2RID_NUM_PER_REG;

		edma_debug("Configure QID2RID(%d) reg:0x%x to 0x%x\n",
				i, EDMA_QID2RID_TABLE_MEM(reg_index), data);
	}

	/*
	 * Map PPE multicast queues to the first Rx ring.
	 */
	desc_index = (egc->rxdesc_ring_start & EDMA_RX_RING_ID_MASK);
	for (i = EDMA_CPU_PORT_MC_QID_MIN;
		i <= EDMA_CPU_PORT_MC_QID_MAX;
			i += EDMA_QID2RID_NUM_PER_REG) {
		reg_index = i/EDMA_QID2RID_NUM_PER_REG;
		data = EDMA_RX_RING_ID_QUEUE0_SET(desc_index) |
			EDMA_RX_RING_ID_QUEUE1_SET(desc_index) |
			EDMA_RX_RING_ID_QUEUE2_SET(desc_index) |
			EDMA_RX_RING_ID_QUEUE3_SET(desc_index);

		edma_reg_write(EDMA_QID2RID_TABLE_MEM(reg_index), data);

		edma_debug("Configure QID2RID(%d) reg:0x%x to 0x%x\n",
				i, EDMA_QID2RID_TABLE_MEM(reg_index), data);
	}
}

/*
 * edma_cfg_rx_rings_to_rx_fill_mapping()
 *	Configure Rx rings to Rx fill mapping
 */
static void edma_cfg_rx_rings_to_rx_fill_mapping(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	/*
	 * Set RXDESC2FILL_MAP_xx reg.
	 * 3 registers hold the Rxfill mapping for total 24 Rxdesc rings.
	 * 3 bits holds the rx fill ring mapping for each of the
	 * rx descriptor ring.
	 */
	edma_reg_write(EDMA_REG_RXDESC2FILL_MAP_0, 0);
	edma_reg_write(EDMA_REG_RXDESC2FILL_MAP_1, 0);
	edma_reg_write(EDMA_REG_RXDESC2FILL_MAP_2, 0);

	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		uint32_t data, reg, ring_id;
		struct edma_rxdesc_ring *rxdesc_ring = &egc->rxdesc_rings[i];

		ring_id = rxdesc_ring->ring_id;
		if ((ring_id >= 0) && (ring_id <= 9)) {
			reg = EDMA_REG_RXDESC2FILL_MAP_0;
		} else if ((ring_id >= 10) && (ring_id <= 19)) {
			reg = EDMA_REG_RXDESC2FILL_MAP_1;
		} else {
			reg = EDMA_REG_RXDESC2FILL_MAP_2;
		}

		edma_debug("Configure RXDESC:%u to use RXFILL:%u\n",
						ring_id,
						rxdesc_ring->rxfill->ring_id);

		/*
		 * Set the Rx fill ring number in the
		 * mapping register.
		 */
		data = edma_reg_read(reg);
		data |= (rxdesc_ring->rxfill->ring_id &
				EDMA_RXDESC2FILL_MAP_RXDESC_MASK) <<
				((ring_id % 10) * 3);
		edma_reg_write(reg, data);
	}

	edma_debug("EDMA_REG_RXDESC2FILL_MAP_0: 0x%x\n", edma_reg_read(EDMA_REG_RXDESC2FILL_MAP_0));
	edma_debug("EDMA_REG_RXDESC2FILL_MAP_1: 0x%x\n", edma_reg_read(EDMA_REG_RXDESC2FILL_MAP_1));
	edma_debug("EDMA_REG_RXDESC2FILL_MAP_2: 0x%x\n", edma_reg_read(EDMA_REG_RXDESC2FILL_MAP_2));
}

/*
 * edma_cfg_rx_rings_enable()
 *	API to enable Rx and Rxfill rings
 */
void edma_cfg_rx_rings_enable(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	/*
	 * Enable Rx rings
	 */
	for (i = egc->rxdesc_ring_start; i < egc->rxdesc_ring_end; i++) {
		uint32_t data;

		data = edma_reg_read(EDMA_REG_RXDESC_CTRL(i));
		data |= EDMA_RXDESC_RX_EN;
		edma_reg_write(EDMA_REG_RXDESC_CTRL(i), data);
	}

	for (i = egc->rxfill_ring_start; i < egc->rxfill_ring_end; i++) {
		uint32_t data;

		data = edma_reg_read(EDMA_REG_RXFILL_RING_EN(i));
		data |= EDMA_RXFILL_RING_EN;
		edma_reg_write(EDMA_REG_RXFILL_RING_EN(i), data);
	}
}

/*
 * edma_cfg_rx_rings_disable()
 *	API to disable Rx and Rxfill rings
 */
void edma_cfg_rx_rings_disable(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	/*
	 * Disable Rx rings
	 */
	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		uint32_t data;
		struct edma_rxdesc_ring *rxdesc_ring = NULL;

		rxdesc_ring = &egc->rxdesc_rings[i];
		data = edma_reg_read(EDMA_REG_RXDESC_CTRL(rxdesc_ring->ring_id));
		data &= ~EDMA_RXDESC_RX_EN;
		edma_reg_write(EDMA_REG_RXDESC_CTRL(rxdesc_ring->ring_id), data);
	}

	/*
	 * Disable RxFill Rings
	 */
	for (i = 0; i < egc->num_rxfill_rings; i++) {
		uint32_t data;
		struct edma_rxfill_ring *rxfill_ring = NULL;

		rxfill_ring = &egc->rxfill_rings[i];
		data = edma_reg_read(EDMA_REG_RXFILL_RING_EN(rxfill_ring->ring_id));
		data &= ~EDMA_RXFILL_RING_EN;
		edma_reg_write(EDMA_REG_RXFILL_RING_EN(rxfill_ring->ring_id), data);
	}
}

/*
 * edma_cfg_rx_mapping()
 *	API to setup RX ring mapping
 */
void edma_cfg_rx_mapping(struct edma_gbl_ctx *egc)
{
	edma_cfg_rx_qid_to_rx_desc_ring_mapping(egc);
	edma_cfg_rx_rings_to_rx_fill_mapping(egc);
}

/*
 * edma_cfg_rx_rings_setup()
 *	Allocate/setup resources for EDMA rings
 */
static int edma_cfg_rx_rings_setup(struct edma_gbl_ctx *egc)
{
	uint32_t queue_id = EDMA_PORT_QUEUE_START;
	int32_t i;

	/*
	 * Allocate Rx fill ring descriptors
	 */
	for (i = 0; i < egc->num_rxfill_rings; i++) {
		int32_t ret;
		struct edma_rxfill_ring *rxfill_ring = NULL;

		rxfill_ring = &egc->rxfill_rings[i];
		rxfill_ring->count = EDMA_RX_RING_SIZE;
		rxfill_ring->ring_id = egc->rxfill_ring_start + i;
		rxfill_ring->alloc_size = dp_global_ctx.rx_buf_size;

		ret = edma_cfg_rx_fill_ring_setup(rxfill_ring);
		if (ret != 0) {
			edma_err("Error in setting up %d rxfill ring. ret: %d",
					 rxfill_ring->ring_id, ret);
			while (--i >= 0) {
				edma_cfg_rx_fill_ring_cleanup(egc,
					&egc->rxfill_rings[i]);
			}

			return -ENOMEM;
		}
	}

	/*
	 * Allocate RxDesc ring descriptors
	 */
	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		uint32_t index, word_idx, bit_idx;
		int32_t ret;
		struct edma_rxdesc_ring *rxdesc_ring = NULL;

		rxdesc_ring = &egc->rxdesc_rings[i];
		rxdesc_ring->count = EDMA_RX_RING_SIZE;
		rxdesc_ring->ring_id = egc->rxdesc_ring_start + i;

		if (queue_id > EDMA_PORT_QUEUE_END) {
			edma_err("Invalid queue_id: %d\n", queue_id);
			while (--i >= 0) {
				edma_cfg_rx_desc_ring_cleanup(egc, &egc->rxdesc_rings[i]);
			}

			goto rxdesc_mem_alloc_fail;
		}

		/*
		 * MAP Rx descriptor ring to PPE queues.
		 *
		 * TODO:
		 * Currently one Rx descriptor ring can get mapped to only
		 * single PPE queue. Multiple queues getting mapped to the
		 * single Rx descriptor ring is not yet supported.
		 * In future, we can support this by getting the Rx descriptor
		 * ring to queue mapping from the dtsi.
		 */
		word_idx = (queue_id / (EDMA_BITS_IN_WORD - 1));
		bit_idx = (queue_id % EDMA_BITS_IN_WORD);
		egc->rxdesc_ring_to_queue_bm[i][word_idx] = 1 << bit_idx;
		queue_id++;

		/*
		 * Create a mapping between RX Desc ring and Rx fill ring.
		 * Number of fill rings are lesser than the descriptor rings
		 * Share the fill rings across descriptor rings.
		 */
		index = egc->rxfill_ring_start + (i % egc->num_rxfill_rings);
		rxdesc_ring->rxfill = &egc->rxfill_rings[index - egc->rxfill_ring_start];

		ret = edma_cfg_rx_desc_ring_setup(rxdesc_ring);
		if (ret != 0) {
			edma_err("Error in setting up %d rxdesc ring. ret: %d",
					 rxdesc_ring->ring_id, ret);
			while (--i >= 0) {
				edma_cfg_rx_desc_ring_cleanup(egc, &egc->rxdesc_rings[i]);
			}

			goto rxdesc_mem_alloc_fail;
		}
	}

	return 0;

rxdesc_mem_alloc_fail:
	for (i = 0; i < egc->num_rxfill_rings; i++) {
		edma_cfg_rx_fill_ring_cleanup(egc, &egc->rxfill_rings[i]);
	}

	return -ENOMEM;
}

/*
 * edma_cfg_rx_rings_alloc()
 *	Allocate EDMA Rx rings
 */
int32_t edma_cfg_rx_rings_alloc(struct edma_gbl_ctx *egc)
{
	egc->rxfill_rings = kzalloc((sizeof(struct edma_rxfill_ring) *
				egc->num_rxfill_rings), GFP_KERNEL);
	if (!egc->rxfill_rings) {
		edma_err("Error in allocating rxfill ring\n");
		return -ENOMEM;
	}

	egc->rxdesc_rings = kzalloc((sizeof(struct edma_rxdesc_ring) *
				egc->num_rxdesc_rings), GFP_KERNEL);
	if (!egc->rxdesc_rings) {
		edma_err("Error in allocating rxdesc ring\n");
		goto rxdesc_ring_alloc_fail;
	}

	edma_info("RxDesc:%u (%u-%u) RxFill:%u (%u-%u)\n",
		egc->num_rxdesc_rings, egc->rxdesc_ring_start,
		(egc->rxdesc_ring_start + egc->num_rxdesc_rings - 1),
		egc->num_rxfill_rings, egc->rxfill_ring_start,
		(egc->rxfill_ring_start + egc->num_rxfill_rings - 1));

	egc->rxdesc_ring_to_queue_bm = kzalloc(egc->num_rxdesc_rings * sizeof(uint32_t) * \
			 EDMA_RING_MAPPED_QUEUE_BM_WORD_COUNT, GFP_KERNEL);
	if (!egc->rxdesc_ring_to_queue_bm) {
		edma_err("Error in allocating mapped queue area for Rxdesc rings\n");
		goto rx_rings_mapped_queue_alloc_failed;
	}

	if (edma_cfg_rx_rings_setup(egc)) {
		edma_err("Error in setting up rx rings\n");
		goto rx_rings_setup_fail;
	}

	/*
	 * Reset Rx descriptor ring mapped queue's configurations
	 */
	if (edma_cfg_rx_desc_ring_reset_queue_config(egc)) {
		edma_err("Error in resetting the Rx descriptor rings configurations\n");
		edma_cfg_rx_rings_cleanup(egc);
		return -EINVAL;
	}

	return 0;

rx_rings_setup_fail:
	kfree(egc->rxdesc_ring_to_queue_bm);
	egc->rxdesc_ring_to_queue_bm = NULL;
rx_rings_mapped_queue_alloc_failed:
	kfree(egc->rxdesc_rings);
	egc->rxdesc_rings = NULL;
rxdesc_ring_alloc_fail:
	kfree(egc->rxfill_rings);
	egc->rxfill_rings = NULL;
	return -ENOMEM;
}

/*
 * edma_cfg_rx_rings_cleanup()
 *	Cleanup EDMA rings
 */
void edma_cfg_rx_rings_cleanup(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	/*
	 * Free Rx fill ring descriptors
	 */
	for (i = 0; i < egc->num_rxfill_rings; i++) {
		edma_cfg_rx_fill_ring_cleanup(egc, &egc->rxfill_rings[i]);
	}

	/*
	 * Free Rx completion ring descriptors
	 */
	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		edma_cfg_rx_desc_ring_cleanup(egc, &egc->rxdesc_rings[i]);
	}

	kfree(egc->rxfill_rings);
	kfree(egc->rxdesc_rings);
	egc->rxfill_rings = NULL;
	egc->rxdesc_rings = NULL;

	if (egc->rxdesc_ring_to_queue_bm) {
		kfree(egc->rxdesc_ring_to_queue_bm);
		egc->rxdesc_ring_to_queue_bm = NULL;
	}
}

/*
 * edma_cfg_rx_rings()
 *	Configure EDMA rings
 */
void edma_cfg_rx_rings(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	/*
	 * Configure RXFILL rings
	 */
	for (i = 0; i < egc->num_rxfill_rings; i++) {
		edma_cfg_rx_fill_ring_configure(egc, &egc->rxfill_rings[i]);
	}

	/*
	 * Configure RXDESC ring
	 */
	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		edma_cfg_rx_desc_ring_configure(egc, &egc->rxdesc_rings[i]);
	}
}

/*
 * edma_cfg_rx_napi_disable()
 *	Disable RX NAPI
 */
void edma_cfg_rx_napi_disable(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		struct edma_rxdesc_ring *rxdesc_ring;

		rxdesc_ring = &egc->rxdesc_rings[i];

		if (!rxdesc_ring->napi_added) {
			continue;
		}

		napi_disable(&rxdesc_ring->napi);
	}
}

/*
 * edma_cfg_rx_napi_enable()
 *	Enable RX NAPI
 */
void edma_cfg_rx_napi_enable(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		struct edma_rxdesc_ring *rxdesc_ring;

		rxdesc_ring = &egc->rxdesc_rings[i];

		if (!rxdesc_ring->napi_added) {
			continue;
		}

		napi_enable(&rxdesc_ring->napi);
	}
}

/*
 * edma_cfg_rx_napi_delete()
 *	Delete RX NAPI
 */
void edma_cfg_rx_napi_delete(struct edma_gbl_ctx *egc)
{
	uint32_t i;

	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		struct edma_rxdesc_ring *rxdesc_ring;

		rxdesc_ring = &egc->rxdesc_rings[i];

		if (!rxdesc_ring->napi_added) {
			continue;
		}

		netif_napi_del(&rxdesc_ring->napi);
		rxdesc_ring->napi_added = false;
	}
}

/*
 * edma_cfg_rx_napi_add()
 *	RX NAPI add API
 */
void edma_cfg_rx_napi_add(struct edma_gbl_ctx *egc, struct net_device *netdev)
{
	uint32_t i;

	for (i = 0; i < egc->num_rxdesc_rings; i++) {
		struct edma_rxdesc_ring *rxdesc_ring = &egc->rxdesc_rings[i];
		netif_napi_add(netdev, &rxdesc_ring->napi,
				edma_rx_napi_poll, EDMA_RX_NAPI_WORK);
		rxdesc_ring->napi_added = true;
	}
}
