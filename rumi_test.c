/*
 **************************************************************************
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 **************************************************************************
 */

#include <linux/netdevice.h>
#include "nss_dp_dev.h"

#define PPE_BASE_ADDR			0x3a000000
#define PPE_REG_SIZE			0x1000000

#define PPE_IPE_L3_BASE_ADDR		0x200000
#define PPE_L3_VP_PORT_TBL_ADDR		(PPE_IPE_L3_BASE_ADDR + 0x1000)
#define PPE_L3_VP_PORT_TBL_INC		0x10

#define PPE_QUEUE_MANAGER_BASE_ADDR	0x800000
#define PPE_UCAST_QUEUE_MAP_TBL_ADDR	0x10000
#define PPE_UCAST_QUEUE_MAP_TBL_INC	0x10
#define PPE_QM_UQM_TBL			(PPE_QUEUE_MANAGER_BASE_ADDR +\
					 PPE_UCAST_QUEUE_MAP_TBL_ADDR)
#define PPE_UCAST_PRIORITY_MAP_TBL_ADDR	0x42000
#define PPE_QM_UPM_TBL			(PPE_QUEUE_MANAGER_BASE_ADDR +\
					 PPE_UCAST_PRIORITY_MAP_TBL_ADDR)

#define PPE_STP_BASE			0x060100
#define PPE_MAC_ENABLE			0x001000
#define PPE_MAC_SPEED			0x001004
#define PPE_MAC_MIB_CTL			0x001034

#define PPE_TRAFFIC_MANAGER_BASE_ADDR	0x400000

#define PPE_L0_FLOW_PORT_MAP_TBL_ADDR	0x8000
#define PPE_L0_FLOW_PORT_MAP_TBL_INC	0x10
#define PPE_L0_FLOW_PORT_MAP_TBL	(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L0_FLOW_PORT_MAP_TBL_ADDR)

#define PPE_L0_FLOW_MAP_TBL_ADDR	0x2000
#define PPE_L0_FLOW_MAP_TBL_INC		0x10
#define PPE_L0_FLOW_MAP_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L0_FLOW_MAP_TBL_ADDR)

#define PPE_L1_FLOW_PORT_MAP_TBL_ADDR	0x46000
#define PPE_L1_FLOW_PORT_MAP_TBL_INC	0x10
#define PPE_L1_FLOW_PORT_MAP_TBL	(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L1_FLOW_PORT_MAP_TBL_ADDR)

#define PPE_L1_FLOW_MAP_TBL_ADDR	0x40000
#define PPE_L1_FLOW_MAP_TBL_INC		0x10
#define PPE_L1_FLOW_MAP_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L1_FLOW_MAP_TBL_ADDR)

#define PPE_L0_C_SP_CFG_TBL_ADDR	0x4000
#define PPE_L0_C_SP_CFG_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L0_C_SP_CFG_TBL_ADDR)

#define PPE_L1_C_SP_CFG_TBL_ADDR	0x42000
#define PPE_L1_C_SP_CFG_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L1_C_SP_CFG_TBL_ADDR)

#define PPE_L0_E_SP_CFG_TBL_ADDR	0x6000
#define PPE_L0_E_SP_CFG_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L0_E_SP_CFG_TBL_ADDR)

#define PPE_L1_E_SP_CFG_TBL_ADDR		0x44000
#define PPE_L1_E_SP_CFG_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L1_E_SP_CFG_TBL_ADDR)

/*
 * ppe_reg_read()
 */
static inline void ppe_reg_read(u32 reg, u32 *val)
{
	*val = readl((void *)(ctx.ppe_base + reg));
}

/*
 * ppe_reg_write()
 */
static inline void ppe_reg_write(u32 reg, u32 val)
{
	writel(val, (void *)(ctx.ppe_base + reg));
}

/*
 * ppe_vp_port_tbl_set()
 */
static void ppe_vp_port_tbl_set(int port, int vsi)
{
	u32 addr = PPE_L3_VP_PORT_TBL_ADDR + port * PPE_L3_VP_PORT_TBL_INC;
	ppe_reg_write(addr, 0x0);
	ppe_reg_write(addr + 0x4 , 1 << 9 | vsi << 10);
	ppe_reg_write(addr + 0x8, 0x0);
}

/*
 * ppe_ucast_queue_map_tbl_queue_id_set()
 */
static void ppe_ucast_queue_map_tbl_queue_id_set(int queue, int port)
{
	uint32_t val;
	ppe_reg_read(PPE_QM_UQM_TBL + port * PPE_UCAST_QUEUE_MAP_TBL_INC, &val);
	val |= queue << 4;
	ppe_reg_write(PPE_QM_UQM_TBL + port * PPE_UCAST_QUEUE_MAP_TBL_INC, val);
}

/*
 * vsi_setup()
 */
static void vsi_setup(int vsi, uint8_t group_mask)
{
	uint32_t val = (group_mask << 24 | group_mask << 16 | group_mask << 8
							    | group_mask);

	/* Set mask */
	ppe_reg_write(0x061800 + vsi * 0x10, val);

	/*  new addr lrn en | station move lrn en */
	ppe_reg_write(0x061804 + vsi * 0x10, 0x9);
}

/*
 * gmac_port_enable()
 */
static void gmac_port_enable(int port)
{
	ppe_reg_write(PPE_MAC_ENABLE + 0x200 * port, 0x13);
	ppe_reg_write(PPE_MAC_SPEED + 0x200 * port, 0x2);
	ppe_reg_write(PPE_MAC_MIB_CTL + 0x200 * port, 0x1);
}

/*
 * ppe_flow_port_map_tbl_port_num_set()
 */
static void ppe_flow_port_map_tbl_port_num_set(int queue, int port)
{
	ppe_reg_write(PPE_L0_FLOW_PORT_MAP_TBL +
			queue * PPE_L0_FLOW_PORT_MAP_TBL_INC, port);
	ppe_reg_write(PPE_L1_FLOW_PORT_MAP_TBL +
			port * PPE_L1_FLOW_PORT_MAP_TBL_INC, port);
}

/*
 * ppe_flow_map_tbl_set()
 */
static void ppe_flow_map_tbl_set(int queue, int port)
{
	uint32_t val = port | 0x401000; /* c_drr_wt = 1, e_drr_wt = 1 */
	ppe_reg_write(PPE_L0_FLOW_MAP_TBL + queue * PPE_L0_FLOW_MAP_TBL_INC,
									val);

	val = port | 0x100400; /* c_drr_wt = 1, e_drr_wt = 1 */
	ppe_reg_write(PPE_L1_FLOW_MAP_TBL + port * PPE_L1_FLOW_MAP_TBL_INC,
									val);
}

/*
 * ppe_c_sp_cfg_tbl_drr_id_set
 */
static void ppe_c_sp_cfg_tbl_drr_id_set(int id)
{
	ppe_reg_write(PPE_L0_C_SP_CFG_TBL + id * 0x80, id * 2);
	ppe_reg_write(PPE_L1_C_SP_CFG_TBL + id * 0x80, id * 2);
}

/*
 * ppe_e_sp_cfg_tbl_drr_id_set
 */
static void ppe_e_sp_cfg_tbl_drr_id_set(int id)
{
	ppe_reg_write(PPE_L0_E_SP_CFG_TBL + id * 0x80, id * 2 + 1);
	ppe_reg_write(PPE_L1_E_SP_CFG_TBL + id * 0x80, id * 2 + 1);
}

/*
 * rumi_test_init()
 */
void rumi_test_init(void)
{
	int i;
	uint32_t queue;

	/*
	 * Get the PPE base address
	 */
	ctx.ppe_base = (u32)ioremap_nocache(PPE_BASE_ADDR, PPE_REG_SIZE);
	if (!ctx.ppe_base) {
		pr_info("NSS DP can't get PPE base address\n");
		return -ENOMEM;
	}

	/* Port4 Port5, Port6 port mux configuration */
	ppe_reg_write(0x10, 0x3b);

	/* disable clock gating */
	ppe_reg_write(0x000008, 0x0);

	/* flow ctrl disable */
	ppe_reg_write(0x200368, 0xc88);

#ifdef RUMI_BRIDGED_FLOW
	ppe_vp_port_tbl_set(1, 2);
	ppe_vp_port_tbl_set(2, 2);
	ppe_vp_port_tbl_set(3, 2);
	ppe_vp_port_tbl_set(4, 2);

#else
	ppe_vp_port_tbl_set(1, 2);
	ppe_vp_port_tbl_set(2, 3);
	ppe_vp_port_tbl_set(3, 4);
	ppe_vp_port_tbl_set(4, 5);
#endif

	/* Unicast priority map */
	ppe_reg_write(PPE_QM_UPM_TBL, 0);

	/* Port0 - 7 unicast queue settings */
	for (i = 0; i < 8; i++) {
		if (i == 0)
			queue = 0;
		else
			queue = ((i * 0x10) + 0x70);

		ppe_ucast_queue_map_tbl_queue_id_set(queue, i);
		ppe_flow_port_map_tbl_port_num_set(queue, i);
		ppe_flow_map_tbl_set(queue, i);
		ppe_c_sp_cfg_tbl_drr_id_set(i);
		ppe_e_sp_cfg_tbl_drr_id_set(i);
	}

	/* Port0 multicast queue */
	ppe_reg_write(0x409000, 0x00000000);
	ppe_reg_write(0x403000, 0x00401000);

	/* Port1 - 7 multicast queue */
	for (i = 1; i < 8; i++) {
		ppe_reg_write(0x409100 + (i - 1) * 0x40, i);
		ppe_reg_write(0x403100 + (i - 1) * 0x40, 0x401000 | i);
	}

	/*
	 * Port0 - Port7 learn enable and isolation port bitmap and TX_EN
	 * Here please pay attention on bit16 (TX_EN) is not set on port7
	 */
	ppe_reg_write(0x060300, 0x37f09);
	ppe_reg_write(0x060304, 0x37f09);
	ppe_reg_write(0x060308, 0x37f09);
	ppe_reg_write(0x06030c, 0x37f09);
	ppe_reg_write(0x060310, 0x37f09);
	ppe_reg_write(0x060314, 0x37f09);
	ppe_reg_write(0x060318, 0x37f09);
	ppe_reg_write(0x06031c, 0x27f09);

	/* Global learning */
	ppe_reg_write(0x060038, 0xc0);

#ifdef RUMI_BRIDGED_FLOW
	vsi_setup(2, 0x1f);
#else
	vsi_setup(2, 0x03);
	vsi_setup(3, 0x05);
	vsi_setup(4, 0x09);
	vsi_setup(5, 0x11);
#endif

	/* Port 0-7 STP */
	for (i = 0; i < 8; i++)
		ppe_reg_write(PPE_STP_BASE + 0x4 * i, 0x3);

	/* Port 0-5 enable */
	for (i = 0; i < 6; i++)
		gmac_port_enable(i);
}

