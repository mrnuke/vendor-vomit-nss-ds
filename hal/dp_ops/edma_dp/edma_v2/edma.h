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

#ifndef __EDMA_H__
#define __EDMA_H__

#include <linux/netdevice.h>
#include "edma_rx.h"
#include "edma_tx.h"
#include <nss_dp_arch.h>

#define EDMA_BUF_SIZE			2000
#define EDMA_DEVICE_NODE_NAME		"edma"
#define EDMA_RX_RING_SIZE		256
#define EDMA_TX_RING_SIZE		256
#define EDMA_RX_RING_SIZE_MASK		(EDMA_RX_RING_SIZE - 1)
#define EDMA_TX_RING_SIZE_MASK		(EDMA_TX_RING_SIZE - 1)
#define EDMA_START_GMACS		NSS_DP_HAL_START_IFNUM
#define EDMA_MAX_GMACS			NSS_DP_HAL_MAX_PORTS

#define EDMA_MAX_TXCMPL_RINGS		32	/* Max TxCmpl rings */
#define EDMA_MAX_RXDESC_RINGS		24	/* Max RxDesc rings */
#define EDMA_MAX_TXDESC_RINGS		32	/* Max TxDesc rings */
#define EDMA_MAX_RXFILL_RINGS		8	/* Max RxFill rings */

/*
 * EDMA private data structure
 */
struct edma_gbl_ctx {
	struct net_device *netdev_arr[EDMA_MAX_GMACS];
			/* Net device for each GMAC port */
	struct device_node *device_node;
			/* Device tree node */
	struct platform_device *pdev;
			/* Platform device */
	void __iomem *reg_base;
			/* EDMA base register mapped address */
	struct resource *reg_resource;
			/* Memory resource */
	int32_t active_port_count;
			/* Count of active number of ports */
	bool napi_added;
			/* NAPI flag */

	struct edma_rxfill_ring *rxfill_rings;
			/* Rx Fill Rings, SW is producer */
	struct edma_rxdesc_ring *rxdesc_rings;
			/* Rx Descriptor Rings, SW is consumer */
	struct edma_txdesc_ring *txdesc_rings;
			/* Tx Descriptor Ring, SW is producer */
	struct edma_txcmpl_ring *txcmpl_rings;
			/* Tx complete Ring, SW is consumer */

	uint32_t rxfill_ring_map[EDMA_RXFILL_RING_PER_CORE_MAX][NR_CPUS];
			/* Rx Fill ring per-core mapping from device tree */
	uint32_t rxdesc_ring_map[EDMA_RXDESC_RING_PER_CORE_MAX][NR_CPUS];
			/* Rx Descriptor ring per-core mapping from device tree */
	int32_t tx_to_txcmpl_map[EDMA_MAX_TXDESC_RINGS];
			/* Tx ring to Tx complete ring mapping */
	int32_t tx_map[EDMA_TX_RING_PER_CORE_MAX][NR_CPUS];
			/* Per core Tx ring to core mapping */
	int32_t txcmpl_map[EDMA_TXCMPL_RING_PER_CORE_MAX][NR_CPUS];
			/* Tx complete ring to core mapping */

	uint32_t tx_priority_level;
			/* Tx priority level per port */
	uint32_t rx_priority_level;
			/* Rx priority level per core */
	uint32_t num_txdesc_rings;
			/* Number of TxDesc rings */
	uint32_t txdesc_ring_start;
			/* Id of first TXDESC ring */
	uint32_t txdesc_ring_end;
			/* Id of the last TXDESC ring */
	uint32_t num_txcmpl_rings;
			/* Number of TxCmpl rings */
	uint32_t txcmpl_ring_start;
			/* Id of first TXCMPL ring */
	uint32_t txcmpl_ring_end;
			/* Id of last TXCMPL ring */
	uint32_t num_rxfill_rings;
			/* Number of RxFill rings */
	uint32_t rxfill_ring_start;
			/* Id of first RxFill ring */
	uint32_t rxfill_ring_end;
			/* Id of last RxFill ring */
	uint32_t num_rxdesc_rings;
			/* Number of RxDesc rings */
	uint32_t rxdesc_ring_start;
			/* Id of first RxDesc ring */
	uint32_t rxdesc_ring_end;
			/* Id of last RxDesc ring */
	uint32_t txcmpl_intr[EDMA_MAX_TXCMPL_RINGS];
			/* TxCmpl ring IRQ numbers */
	uint32_t rxfill_intr[EDMA_MAX_RXFILL_RINGS];
			/* Rx fill ring IRQ numbers */
	uint32_t rxdesc_intr[EDMA_MAX_RXDESC_RINGS];
			/* Rx desc ring IRQ numbers */
	uint32_t misc_intr;
			/* Misc IRQ number */

	uint32_t rxfill_intr_mask;
			/* Rx fill ring interrupt mask */
	uint32_t rxdesc_intr_mask;
			/* Rx Desc ring interrupt mask */
	uint32_t txcmpl_intr_mask;
			/* Tx Cmpl ring interrupt mask */
	uint32_t misc_intr_mask;
			/* Misc interrupt interrupt mask */
	uint32_t dp_override_cnt;
			/* Number of interfaces overriden */
	bool edma_initialized;
			/* Flag to check initialization status */
};

#endif	/* __EDMA_H__ */
