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

#ifndef __EDMA_REGS__
#define __EDMA_REGS__

#define EDMA_GENMASK(end, start)	(uint32_t)((((uint64_t)1 << ((end) - (start) + 1)) - 1) << (start))

/*
 * EDMA register offsets
 */
#define EDMA_REG_MAS_CTRL		0x0
#define EDMA_REG_PORT_CTRL		0x4
#define EDMA_REG_VLAN_CTRL		0x8
#define EDMA_REG_RXDESC2FILL_MAP_0	0x14
#define EDMA_REG_RXDESC2FILL_MAP_1	0x18
#define EDMA_REG_RXDESC2FILL_MAP_2	0x1c
#define EDMA_REG_TXQ_CTRL		0x20
#define EDMA_REG_TXQ_CTRL_2		0x24
#define EDMA_REG_TXQ_FC_0		0x28
#define EDMA_REG_TXQ_FC_1		0x30
#define EDMA_REG_TXQ_FC_2		0x34
#define EDMA_REG_TXQ_FC_3		0x38
#define EDMA_REG_RXQ_CTRL		0x3c
#define EDMA_REG_MISC_ERR_QID		0x40
#define EDMA_REG_RXQ_FC_THRE		0x44
#define EDMA_REG_DMAR_CTRL		0x48
#define EDMA_REG_AXIR_CTRL		0x4c
#define EDMA_REG_AXIW_CTRL		0x50
#define EDMA_REG_MIN_MSS		0x54
#define EDMA_REG_LOOPBACK_CTRL		0x58
#define EDMA_REG_MISC_INT_STAT		0x5c
#define EDMA_REG_MISC_INT_MASK		0x60
#define EDMA_REG_DBG_CTRL		0x64
#define EDMA_REG_DBG_DATA		0x68
#define EDMA_REG_TX_TIMEOUT_THRESH	0x6c
#define EDMA_REG_REQ0_FIFO_THRESH	0x80
#define EDMA_REG_WB_OS_THRESH		0x84
#define EDMA_REG_MISC_ERR_QID_REG2	0x88
#define EDMA_REG_TXDESC2CMPL_MAP_0	0x8c
#define EDMA_REG_TXDESC2CMPL_MAP_1	0x90
#define EDMA_REG_TXDESC2CMPL_MAP_2	0x94
#define EDMA_REG_TXDESC2CMPL_MAP_3	0x98
#define EDMA_REG_TXDESC2CMPL_MAP_4	0x9c
#define EDMA_REG_TXDESC2CMPL_MAP_5	0xa0

#define EDMA_REG_TXDESC_BA(n)		(0x1000 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_PROD_IDX(n)	(0x1004 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_CONS_IDX(n)	(0x1008 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_RING_SIZE(n)	(0x100c + (0x1000 * (n)))
#define EDMA_REG_TXDESC_CTRL(n)		(0x1010 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_BA2(n)		(0x1014 + (0x1000 * (n)))

#define EDMA_REG_RXFILL_BA(n)		(0x29000 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_PROD_IDX(n)	(0x29004 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_CONS_IDX(n)	(0x29008 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_RING_SIZE(n)	(0x2900c + (0x1000 * (n)))
#define EDMA_REG_RXFILL_BUFFER1_SIZE(n)	(0x29010 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_FC_THRE(n)	(0x29014 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_UGT_THRE(n)	(0x29018 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_RING_EN(n)	(0x2901c + (0x1000 * (n)))
#define EDMA_REG_RXFILL_DISABLE(n)	(0x29020 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_DISABLE_DONE(n)	(0x29024 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_INT_STAT(n)	(0x31000 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_INT_MASK(n)	(0x31004 + (0x1000 * (n)))

#define EDMA_REG_RXDESC_BA(n)		(0x39000 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_PROD_IDX(n)	(0x39004 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_CONS_IDX(n)	(0x39008 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_RING_SIZE(n)	(0x3900c + (0x1000 * (n)))
#define EDMA_REG_RXDESC_FC_THRE(n)	(0x39010 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_UGT_THRE(n)	(0x39014 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_CTRL(n)		(0x39018 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_BPC(n)		(0x3901c + (0x1000 * (n)))
#define EDMA_REG_RXDESC_DISABLE(n)	(0x39020 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_DISABLE_DONE(n)	(0x39024 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_PREHEADER_BA(n)	(0x39028 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_INT_STAT(n)	(0x59000 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_INT_MASK(n)	(0x59004 + (0x1000 * (n)))

#define EDMA_REG_RX_MOD_TIMER(n)	(0x59008 + (0x1000 * (n)))
#define EDMA_REG_RX_INT_CTRL(n)		(0x5900c + (0x1000 * (n)))

#define EDMA_REG_TXCMPL_BA(n)		(0x79000 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_PROD_IDX(n)	(0x79004 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_CONS_IDX(n)	(0x79008 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_RING_SIZE(n)	(0x7900c + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_UGT_THRE(n)	(0x79010 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_CTRL(n)		(0x79014 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_BPC(n)		(0x79018 + (0x1000 * (n)))

#define EDMA_REG_TX_INT_STAT(n)		(0x99000 + (0x1000 * (n)))
#define EDMA_REG_TX_INT_MASK(n)		(0x99004 + (0x1000 * (n)))
#define EDMA_REG_TX_MOD_TIMER(n)	(0x99008 + (0x1000 * (n)))
#define EDMA_REG_TX_INT_CTRL(n)		(0x9900c + (0x1000 * (n)))

/*
 * QID to RID Table
 */
#define EDMA_QID2RID_TABLE_MEM(q)	(0xb9000 + (0x4 * (q)))

/*
 * EDMA QID2RID configuration
 *
 * TODO: Get port queue ids from dtsi
 */
#define EDMA_PORT_QUEUE_START		0
#define EDMA_PORT_QUEUE_END		3
#define EDMA_PORT_QUEUE_NUM		((EDMA_PORT_QUEUE_END) - (EDMA_PORT_QUEUE_START) + 1)
#define EDMA_PORT_QUEUE_PER_CORE	((EDMA_PORT_QUEUE_NUM)/(CONFIG_NR_CPUS))
#define EDMA_CPU_PORT_MC_QID_MIN	256
#define EDMA_CPU_PORT_MC_QID_MAX	271
#define EDMA_QID2RID_NUM_PER_REG	4
#define EDMA_RSS_HASH_MAX		256
#define EDMA_PORT_PROFILE_ID		0

#define EDMA_RX_RING_ID_QUEUE0_SHIFT	0
#define EDMA_RX_RING_ID_QUEUE0_MASK	EDMA_GENMASK(7, 0)
#define EDMA_RX_RING_ID_QUEUE0_GET(x)	(((x) & EDMA_RX_RING_ID_QUEUE0_MASK) >> EDMA_RX_RING_ID_QUEUE0_SHIFT)
#define EDMA_RX_RING_ID_QUEUE0_SET(x)	(((x) << EDMA_RX_RING_ID_QUEUE0_SHIFT) & EDMA_RX_RING_ID_QUEUE0_MASK)

#define EDMA_RX_RING_ID_QUEUE1_SHIFT	8
#define EDMA_RX_RING_ID_QUEUE1_MASK	EDMA_GENMASK(15, 8)
#define EDMA_RX_RING_ID_QUEUE1_GET(x)	(((x) & EDMA_RX_RING_ID_QUEUE1_MASK) >> EDMA_RX_RING_ID_QUEUE1_SHIFT)
#define EDMA_RX_RING_ID_QUEUE1_SET(x)	(((x) << EDMA_RX_RING_ID_QUEUE1_SHIFT) & EDMA_RX_RING_ID_QUEUE1_MASK)

#define EDMA_RX_RING_ID_QUEUE2_SHIFT	16
#define EDMA_RX_RING_ID_QUEUE2_MASK	EDMA_GENMASK(23, 16)
#define EDMA_RX_RING_ID_QUEUE2_GET(x)	(((x) & EDMA_RX_RING_ID_QUEUE2_MASK) >> EDMA_RX_RING_ID_QUEUE2_SHIFT)
#define EDMA_RX_RING_ID_QUEUE2_SET(x)	(((x) << EDMA_RX_RING_ID_QUEUE2_SHIFT) & EDMA_RX_RING_ID_QUEUE2_MASK)

#define EDMA_RX_RING_ID_QUEUE3_SHIFT	24
#define EDMA_RX_RING_ID_QUEUE3_MASK	EDMA_GENMASK(31, 24)
#define EDMA_RX_RING_ID_QUEUE3_GET(x)	(((x) & EDMA_RX_RING_ID_QUEUE3_MASK) >> EDMA_RX_RING_ID_QUEUE3_SHIFT)
#define EDMA_RX_RING_ID_QUEUE3_SET(x)	(((x) << EDMA_RX_RING_ID_QUEUE3_SHIFT) & EDMA_RX_RING_ID_QUEUE3_MASK)

/*
 * PPE Hash seed and mask for configuring RPS hash map table
 */
#define PPE_HASH_SEED_DEFAULT		0xabbcdefa
#define PPE_HASH_MASK			0xfff
#define PPE_HASH_MIX_V4_SIP		0x13
#define PPE_HASH_MIX_V4_DIP		0xb
#define PPE_HASH_MIX_V4_PROTO		0x13
#define PPE_HASH_MIX_V4_DPORT		0xb
#define PPE_HASH_MIX_V4_SPORT		0x13

#define PPE_HASH_FIN_INNER_OUTER_0	0x205
#define PPE_HASH_FIN_INNER_OUTER_1	0x264
#define PPE_HASH_FIN_INNER_OUTER_2	0x227
#define PPE_HASH_FIN_INNER_OUTER_3	0x245
#define PPE_HASH_FIN_INNER_OUTER_4	0x201

#define PPE_HASH_SIPV6_MIX_0		0x13
#define PPE_HASH_SIPV6_MIX_1		0xb
#define PPE_HASH_SIPV6_MIX_2		0x13
#define PPE_HASH_SIPV6_MIX_3		0xb
#define PPE_HASH_DIPV6_MIX_0		0x13
#define PPE_HASH_DIPV6_MIX_1		0xb
#define PPE_HASH_DIPV6_MIX_2		0x13
#define PPE_HASH_DIPV6_MIX_3		0xb

/*
 * EDMA_REG_PORT_CTRL register
 */
#define EDMA_PORT_PAD_EN			0x1
#define EDMA_PORT_EDMA_EN			0x2

/*
 * EDMA_REG_TXQ_CTRL register
 */
#define EDMA_TXDESC_PF_THRE_MASK		0xf
#define EDMA_TXDESC_PF_THRE_SHIFT		0
#define EDMA_TXCMPL_WB_THRE_MASK		0xf
#define EDMA_TXCMPL_WB_THRE_SHIFT		4
#define EDMA_TXDESC_PKT_SRAM_THRE_MASK		0xff
#define EDMA_TXDESC_PKT_SRAM_THRE_SHIFT		8
#define EDMA_TXCMPL_WB_TIMER_MASK		0xffff
#define EDMA_TXCMPL_WB_TIMER_SHIFT		16

/*
 * EDMA_REG_RXQ_CTRL register
 */
#define EDMA_RXFILL_PF_THRE_MASK		0xf
#define EDMA_RXFILL_PF_THRE_SHIFT		0
#define EDMA_RXDESC_WB_THRE_MASK		0xf
#define EDMA_RXDESC_WB_THRE_SHIFT		4
#define EDMA_RXDESC_WB_TIMER_MASK		0xffff
#define EDMA_RXDESC_WB_TIMER_SHIFT		16

/*
 * EDMA_REG_RX_TX_FULL_QID register
 */
#define EDMA_RX_DESC_FULL_QID_MASK		0xff
#define EDMA_RX_DESC_FULL_QID_SHIFT		0
#define EDMA_TX_CMPL_BUF_FULL_QID_MASK		0xff
#define EDMA_TX_CMPL_BUF_FULL_QID_SHIFT		8
#define EDMA_TX_SRAM_FULL_QID_MASK		0x1f
#define EDMA_TX_SRAM_FULL_QID_SHIFT		16

/*
 * EDMA_REG_RXQ_FC_THRE reister
 */
#define EDMA_RXFILL_FIFO_XOFF_THRE_MASK		0x1f
#define EDMA_RXFILL_FIFO_XOFF_THRE_SHIFT	0
#define EDMA_DESC_FIFO_XOFF_THRE_MASK		0x3f
#define EDMA_DESC_FIFO_XOFF_THRE_SHIFT		16

/*
 * EDMA_REG_DMAR_CTRL register
 */
#define EDMA_DMAR_REQ_PRI_MASK			0x7
#define EDMA_DMAR_REQ_PRI_SHIFT			0
#define EDMA_DMAR_BURST_LEN_MASK		0x1
#define EDMA_DMAR_BURST_LEN_SHIFT		3
#define EDMA_DMAR_TXDATA_OUTSTANDING_NUM_MASK	0x1f
#define EDMA_DMAR_TXDATA_OUTSTANDING_NUM_SHIFT	4
#define EDMA_DMAR_TXDESC_OUTSTANDING_NUM_MASK	0x7
#define EDMA_DMAR_TXDESC_OUTSTANDING_NUM_SHIFT	9
#define EDMA_DMAR_RXFILL_OUTSTANDING_NUM_MASK	0x7
#define EDMA_DMAR_RXFILL_OUTSTANDING_NUM_SHIFT	12

#define EDMA_DMAR_REQ_PRI_SET(x)		(((x) & EDMA_DMAR_REQ_PRI_MASK) \
						<< EDMA_DMAR_REQ_PRI_SHIFT)
#define EDMA_DMAR_TXDATA_OUTSTANDING_NUM_SET(x)	(((x) & EDMA_DMAR_TXDATA_OUTSTANDING_NUM_MASK) \
						<< EDMA_DMAR_TXDATA_OUTSTANDING_NUM_SHIFT)
#define EDMA_DMAR_TXDESC_OUTSTANDING_NUM_SET(x)	(((x) & EDMA_DMAR_TXDESC_OUTSTANDING_NUM_MASK) \
						<< EDMA_DMAR_TXDESC_OUTSTANDING_NUM_SHIFT)
#define EDMA_DMAR_RXFILL_OUTSTANDING_NUM_SET(x)	(((x) & EDMA_DMAR_RXFILL_OUTSTANDING_NUM_MASK) \
						<< EDMA_DMAR_RXFILL_OUTSTANDING_NUM_SHIFT)
#define EDMA_DMAR_BURST_LEN_SET(x)		(((x) & EDMA_DMAR_BURST_LEN_MASK) \
						<< EDMA_DMAR_BURST_LEN_SHIFT)

#define EDMA_BURST_LEN_ENABLE		0

/*
 * EDMA RXDESC base address mask
 */
#define EDMA_RXDESC_BA_MASK			0xffffffff

/*
 * EDMA RXDESC pre-header base address mask
 */
#define EDMA_RXDESC_PREHEADER_BA_MASK		0xffffffff

/*
 * EDMA_REG_AXIW_CTRL_REG
 */
#define EDMA_AXIW_MAX_WR_SIZE_EN		0x400

/*
 * EDMA DISABLE
 */
#define EDMA_DISABLE				0

/*
 * EDMA_REG_TXDESC_PROD_IDX register
 */
#define EDMA_TXDESC_PROD_IDX_MASK		0xffff

/*
 * EDMA_REG_TXDESC_CONS_IDX register
 */
#define EDMA_TXDESC_CONS_IDX_MASK		0xffff

/*
 * EDMA_REG_TXDESC_RING_SIZE register
 */
#define EDMA_TXDESC_RING_SIZE_MASK		0xffff

/*
 * EDMA_REG_TXDESC_CTRL register
 */
#define EDMA_TXDESC_ARB_GRP_ID_MASK		0x3
#define EDMA_TXDESC_ARB_GRP_ID_SHIFT		4
#define EDMA_TXDESC_FC_GRP_ID_MASK		0x7
#define EDMA_TXDESC_FC_GRP_ID_SHIFT		1
#define EDMA_TXDESC_TX_EN			0x1

/*
 * EDMA_REG_TXCMPL_PROD_IDX register
 */
#define EDMA_TXCMPL_PROD_IDX_MASK		0xffff

/*
 * EDMA_REG_TXCMPL_CONS_IDX register
 */
#define EDMA_TXCMPL_CONS_IDX_MASK		0xffff

/*
 * EDMA_REG_TXCMPL_RING_SIZE register
 */
#define EDMA_TXCMPL_RING_SIZE_MASK		0xffff

/*
 * EDMA_REG_TXCMPL_UGT_THRE register
 */
#define EDMA_TXCMPL_LOW_THRE_MASK		0xffff
#define EDMA_TXCMPL_LOW_THRE_SHIFT		0
#define EDMA_TXCMPL_FC_THRE_MASK		0x3f
#define EDMA_TXCMPL_FC_THRE_SHIFT		16

/*
 * EDMA_REG_TXCMPL_CTRL register
 */
#define EDMA_TXCMPL_RET_MODE_BUFF_ADDR		0x0
#define EDMA_TXCMPL_RET_MODE_OPAQUE		0x1

/*
 * EDMA_REG_TX_MOD_TIMER register
 */
#define EDMA_TX_MOD_TIMER_INIT_MASK		0xffff
#define EDMA_TX_MOD_TIMER_INIT_SHIFT		0

/*
 * EDMA_REG_TX_INT_CTRL register
 */
#define EDMA_TX_INT_MASK			0x3

/*
 * EDMA_REG_RXFILL_PROD_IDX register
 */
#define EDMA_RXFILL_PROD_IDX_MASK		0xffff

/*
 * EDMA_REG_RXFILL_CONS_IDX register
 */
#define EDMA_RXFILL_CONS_IDX_MASK		0xffff

/*
 * EDMA_REG_RXFILL_RING_SIZE register
 */
#define EDMA_RXFILL_RING_SIZE_MASK		0xffff

/*
 * EDMA_REG_RXFILL_FC_THRE register
 */
#define EDMA_RXFILL_FC_XON_THRE_MASK		0x7ff
#define EDMA_RXFILL_FC_XON_THRE_SHIFT		12
#define EDMA_RXFILL_FC_XOFF_THRE_MASK		0x7ff
#define EDMA_RXFILL_FC_XOFF_THRE_SHIFT		0

/*
 * EDMA_REG_RXFILL_UGT_THRE register
 */
#define EDMA_RXFILL_LOW_THRE_MASK		0xffff
#define EDMA_RXFILL_LOW_THRE_SHIFT		0

/*
 * EDMA_REG_RXFILL_RING_EN register
 */
#define EDMA_RXFILL_RING_EN			0x1

/*
 * EDMA_REG_RXFILL_INT_MASK register
 */
#define EDMA_RXFILL_INT_MASK			0x1

/*
 * EDMA_REG_RXDESC_PROD_IDX register
 */
#define EDMA_RXDESC_PROD_IDX_MASK		0xffff

/*
 * EDMA_REG_RXDESC_CONS_IDX register
 */
#define EDMA_RXDESC_CONS_IDX_MASK		0xffff

/*
 * EDMA_REG_RXDESC_RING_SIZE register
 */
#define EDMA_RXDESC_RING_SIZE_MASK		0xffff
#define EDMA_RXDESC_PL_OFFSET_MASK		0x1ff
#define EDMA_RXDESC_PL_OFFSET_SHIFT		16
#define EDMA_RXDESC_PL_DEFAULT_VALUE		0

/*
 * EDMA_REG_RXDESC_FC_THRE register
 */
#define EDMA_RXDESC_FC_XON_THRE_MASK		0x7ff
#define EDMA_RXDESC_FC_XON_THRE_SHIFT		12
#define EDMA_RXDESC_FC_XOFF_THRE_MASK		0x7ff
#define EDMA_RXDESC_FC_XOFF_THRE_SHIFT		0

/*
 * EDMA_REG_RXDESC_UGT_THRE register
 */
#define EDMA_RXDESC_LOW_THRE_MASK		0xffff
#define EDMA_RXDESC_LOW_THRE_SHIFT		0

/*
 * EDMA_REG_RXDESC_CTRL register
 */
#define EDMA_RXDESC_STAG_REMOVE_EN		0x8
#define EDMA_RXDESC_CTAG_REMOVE_EN		0x4
#define EDMA_RXDESC_QDISC_EN			0x2
#define EDMA_RXDESC_RX_EN			0x1

/*
 * EDMA_REG_TX_INT_MASK register
 */
#define EDMA_TX_INT_MASK_PKT_INT		0x1
#define EDMA_TX_INT_MASK_UGT_INT		0x2

/*
 * EDMA_REG_RXDESC_INT_STAT register
 */
#define EDMA_RXDESC_INT_STAT_PKT_INT		0x1
#define EDMA_RXDESC_INT_STAT_UGT_INT		0x2

/*
 * EDMA_REG_RXDESC_INT_MASK register
 */
#define EDMA_RXDESC_INT_MASK_PKT_INT		0x1
#define EDMA_RXDESC_INT_MASK_TIMER_INT_DIS	0x2

#define EDMA_MASK_INT_DISABLE			0x0
#define EDMA_MASK_INT_CLEAR			0x0

/*
 * EDMA_REG_RX_MOD_TIMER register
 */
#define EDMA_RX_MOD_TIMER_INIT_MASK		0xffff
#define EDMA_RX_MOD_TIMER_INIT_SHIFT		0

/*
 * EDMA QID2RID register sizes
 */
#define EDMA_QID2RID_DEPTH			0x40
#define EDMA_QID2RID_QUEUES_PER_ENTRY		8

/*
 * TXDESC shift values
 */
#define EDMA_TXDESC_MORE_SHIFT			31
#define EDMA_TXDESC_TSO_EN_SHIFT		30
#define EDMA_TXDESC_PREHEADER_SHIFT		29
#define EDMA_TXDESC_POOL_ID_SHIFT		24
#define EDMA_TXDESC_POOL_ID_MASK		0x1f
#define EDMA_TXDESC_DATA_OFFSET_SHIFT		16
#define EDMA_TXDESC_DATA_OFFSET_MASK		0xff
#define EDMA_TXDESC_DATA_LENGTH_SHIFT		0
#define EDMA_TXDESC_DATA_LENGTH_MASK		0x1ffff

#define EDMA_RING_DMA_MASK			0xffffffff

/*
 * RXDESC shift values
 */
#define EDMA_RXDESC_RX_RXFILL_CNT_MASK		0x000f
#define EDMA_RXDESC_RX_RXFILL_CNT_SHIFT		16

#define EDMA_RXDESC_PKT_SIZE_MASK		0x3fff
#define EDMA_RXDESC_PKT_SIZE_SHIFT		0

#define EDMA_RXDESC_RXD_VALID_MASK		0x1
#define EDMA_RXDESC_RXD_VALID_SHIFT		31

#define EDMA_RXDESC_RING_INT_STATUS_MASK	0x3

#define EDMA_RING_DISABLE			0
#define EDMA_TXCMPL_RING_INT_STATUS_MASK	0x3
#define EDMA_TXCMPL_RETMODE_OPAQUE		0x0
#define EDMA_RXFILL_RING_INT_STATUS_MASK	0x1

/*
 * TODO tune the timer and threshold values
 */
#define EDMA_RXFILL_FIFO_XOFF_THRE		0x3
#define EDMA_RXFILL_PF_THRE			0x3
#define EDMA_RXDESC_WB_THRE			0x0
#define EDMA_RXDESC_WB_TIMER			0x2

#define EDMA_RXDESC_XON_THRE			50
#define EDMA_RXDESC_XOFF_THRE			30
#define EDMA_RXDESC_LOW_THRE			0
#define EDMA_RX_MOD_TIMER_INIT			1000
#define EDMA_RX_NE_INT_EN			0x2

#define EDMA_TXDESC_PF_THRE			0x3
#define EDMA_TXCMPL_WB_THRE			0X0
#define EDMA_TXDESC_PKT_SRAM_THRE		0x20
#define EDMA_TXCMPL_WB_TIMER			0x2

#define EDMA_TX_MOD_TIMER			150

#define EDMA_TX_INITIAL_PROD_IDX		0x0
#define EDMA_TX_NE_INT_EN			0x2

/*
 * EDMA misc error mask
 */
#define EDMA_MISC_AXI_RD_ERR_MASK		0x1
#define EDMA_MISC_AXI_WR_ERR_MASK		0x2
#define EDMA_MISC_RX_DESC_FIFO_FULL_MASK	0x4
#define EDMA_MISC_RX_ERR_BUF_SIZE_MASK		0x8
#define EDMA_MISC_TX_SRAM_FULL_MASK		0x10
#define EDMA_MISC_TX_CMPL_BUF_FULL_MASK		0x20

#define EDMA_MISC_DATA_LEN_ERR_MASK		0x40
#define EDMA_MISC_TX_TIMEOUT_MASK		0x80

/*
 * EDMA txdesc2cmpl map
 */
#define EDMA_TXDESC2CMPL_MAP_TXDESC_MASK	0x1F

/*
 * EDMA rxdesc2fill map
 */
#define EDMA_RXDESC2FILL_MAP_RXDESC_MASK	0x7

#endif	/* __EDMA_REGS__ */
