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

#ifndef __EDMA_TX_H__
#define __EDMA_TX_H__

#define EDMA_GET_DESC(R, i, type)	(&(((type *)((R)->desc))[(i)]))
#define EDMA_GET_PDESC(R, i, type)	(&(((type *)((R)->pdesc))[(i)]))
#define EDMA_GET_SDESC(R, i, type)	(&(((type *)((R)->sdesc))[(i)]))
#define EDMA_TXCMPL_DESC(R, i)		EDMA_GET_DESC(R, i, struct edma_txcmpl_desc)
#define EDMA_TXDESC_PRI_DESC(R, i)	EDMA_GET_PDESC(R, i, struct edma_pri_txdesc)
#define EDMA_TXDESC_SEC_DESC(R, i)	EDMA_GET_SDESC(R, i, struct edma_sec_txdesc)

#define EDMA_MAX_TXCMPL_RINGS		32	/* Max TxCmpl rings */
#define EDMA_MAX_TXDESC_RINGS		32	/* Max TxDesc rings */

#define EDMA_TXCMPL_RING_PER_CORE_MAX	6
#define EDMA_TX_MAX_PRIORITY_LEVEL	1

#define EDMA_TX_RING_SIZE		256
#define EDMA_TX_RING_SIZE_MASK		(EDMA_TX_RING_SIZE - 1)

#define EDMA_TX_RING_PER_CORE_MAX	(EDMA_TX_MAX_PRIORITY_LEVEL * EDMA_MAX_GMACS)

/*
 * edma_pri_txdesc
 *	EDMA primary TX descriptor.
 */
struct edma_pri_txdesc {
	uint32_t word0;		/* Low 32-bit of buffer address */
	uint32_t word1;		/* Buffer recycling, PTP tag flag, PRI valid flag */
	uint32_t word2;		/* Low 32-bit of opaque value */
	uint32_t word3;		/* High 32-bit of opaque value */
	uint32_t word4;		/* Source/Destination port info */
	uint32_t word5;		/* VLAN offload, csum_mode, ip_csum_en, tso_en, data length */
	uint32_t word6;		/* MSS/hash_value/PTP tag, data offset */
	uint32_t word7;		/* L4/L3 offset, PROT type, L2 type, CVLAN/SVLAN tag, service code */
};

/*
 * edma_sec_txdesc
 *	EDMA secondary TX descriptor.
 */
struct edma_sec_txdesc {
	uint32_t word0;		/* Reserved */
	uint32_t word1;		/* Custom csum offset, payload offset, TTL/NAT action */
	uint32_t word2;		/* NAPT translated port, DSCP value, TTL value */
	uint32_t word3;		/* Flow index value and valid flag */
	uint32_t word4;		/* Reserved */
	uint32_t word5;		/* Reserved */
	uint32_t word6;		/* CVLAN/SVLAN command */
	uint32_t word7;		/* CVLAN/SVLAN tag value */
};

/*
 * edma_txcmpl_desc
 *	EDMA TX complete descriptor.
 */
struct edma_txcmpl_desc {
	uint32_t word0;		/* Low 32-bit opaque value */
	uint32_t word1;		/* High 32-bit opaque value */
	uint32_t word2;		/* More fragment, transmit ring id, pool id */
	uint32_t word3;		/* Error indications */
};

/*
 * edma_txdesc_ring
 *	EDMA TX descriptor ring.
 */
struct edma_txdesc_ring {
	uint32_t prod_idx;		/* Producer index */
	uint32_t avail_desc;		/* Number of available descriptor to process */
	uint32_t id;			/* TXDESC ring number */
	struct edma_pri_txdesc *pdesc;	/* Primary descriptor ring virtual address */
	dma_addr_t pdma;		/* Primary descriptor ring physical address */
	struct edma_sec_txdesc *sdesc;	/* Secondary descriptor ring virtual address */
	dma_addr_t sdma;		/* Secondary descriptor ring physical address */
	uint32_t count;			/* Number of descriptors */
};

/*
 * edma_txcmpl_ring
 *	EDMA TX complete ring.
 */
struct edma_txcmpl_ring {
	struct napi_struct napi;	/* NAPI structure */
	uint32_t cons_idx;		/* Consumer index */
	uint32_t avail_pkt;		/* Number of available packets to process */
	struct edma_txcmpl_desc *desc;	/* Descriptor ring virtual address */
	uint32_t id;			/* TXCMPL ring number */
	dma_addr_t dma;			/* Descriptor ring physical address */
	uint32_t count;			/* Number of descriptors in the ring */
	bool napi_added;		/* Flag to indicate NAPI add status */
};

#endif	/* __EDMA_TX_H__ */
