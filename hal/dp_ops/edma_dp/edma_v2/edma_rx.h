/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
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

#ifndef __EDMA_RX_H__
#define __EDMA_RX_H__

#define EDMA_RXFILL_RING_PER_CORE_MAX	1
#define EDMA_RXDESC_RING_PER_CORE_MAX	1

#define EDMA_GET_DESC(R, i, type)	(&((((type) *)((R)->desc))[(i)]))
#define EDMA_GET_PDESC(R, i, type)	(&((((type) *)((R)->pdesc))[(i)]))
#define EDMA_GET_SDESC(R, i, type)	(&((((type) *)((R)->sdesc))[(i)]))
#define EDMA_RXFILL_DESC(R, i)		EDMA_GET_DESC(R, i, struct edma_rxfill_desc)
#define EDMA_RXDESC_PRI_DESC(R, i)	EDMA_GET_PDESC(R, i, struct edma_rxdesc_desc)
#define EDMA_RXDESC_SEC_DESC(R, i)	EDMA_GET_SDESC(R, i, struct edma_rxdesc_sec_desc)

/*
 * Rx descriptor
 */
struct edma_rxdesc_desc {
	uint32_t word0;		/* Contains buffer address */
	uint32_t word1;		/* Contains more bit, priority bit, service code */
	uint32_t word2;		/* Contains opaque */
	uint32_t word3;		/* Contains opaque high bits */
	uint32_t word4;		/* Contains destination and source information */
	uint32_t word5;		/* Contains WiFi QoS, data length */
	uint32_t word6;		/* Contains hash value, check sum status */
	uint32_t word7;		/* Contains DSCP, packet offsets */
};

/*
 * Rx secondary descriptor
 */
struct edma_rxdesc_sec_desc {
	uint32_t word0;		/* Contains timestamp */
	uint32_t word1;		/* Contains secondary checksum status */
	uint32_t word2;		/* Contains QoS tag */
	uint32_t word3;		/* Contains flow index details */
	uint32_t word4;		/* Contains secondary packet offsets */
	uint32_t word5;		/* Contains multicast bit, checksum */
	uint32_t word6;		/* Contains SVLAN, CVLAN */
	uint32_t word7;		/* Contains secondary SVLAN, CVLAN */
};

/*
 * RxFill descriptor
 */
struct edma_rxfill_desc {
	uint32_t word0;		/* Contains buffer address */
	uint32_t word1;		/* Contains buffer size */
	uint32_t word2;		/* Contains opaque */
	uint32_t word3;		/* Contains opaque high bits */
};

/*
 * RxFill ring
 */
struct edma_rxfill_ring {
	uint32_t ring_id;		/* RXFILL ring number */
	uint32_t count;			/* number of descriptors in the ring */
	uint32_t prod_idx;		/* Ring producer index */
	struct edma_rxfill_desc *desc;	/* descriptor ring virtual address */
	dma_addr_t dma;			/* descriptor ring physical address */
};

/*
 * RxDesc ring
 */
struct edma_rxdesc_ring {
	struct napi_struct napi;	/* Napi structure */
	uint32_t ring_id;		/* RXDESC ring number */
	uint32_t count;			/* number of descriptors in the ring */
	uint32_t avail_pkt;		/* Packets available to be processed */
	uint32_t cons_idx;		/* Ring consumer index */
	struct edma_rxdesc_desc *pdesc;
					/* Primary descriptor ring virtual address */
	struct edma_rxdesc_sec_desc *sdesc;
					/* Secondary descriptor ring virtual address */
	struct edma_rxfill_ring *rxfill;
					/* RXFILL ring used */
	bool napi_added;		/* Flag to indicate NAPI add status */
	dma_addr_t pdma;		/* Primary descriptor ring physical address */
	dma_addr_t sdma;		/* Secondary descriptor ring physical address */
};

#endif	/* __EDMA_RX_H__ */
