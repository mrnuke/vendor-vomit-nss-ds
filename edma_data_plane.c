/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <linux/interrupt.h>
#include "nss_dp_dev.h"

/*
 * edma_handle_irq()
 *	Process IRQ and schedule napi
 */
static irqreturn_t edma_handle_irq(int irq, void *ctx)
{
	return IRQ_HANDLED;
}

/*
 * edma_if_open()
 *	Do slow path data plane open
 */
static int edma_if_open(void *app_data, uint32_t tx_desc_ring,
					uint32_t rx_desc_ring, uint32_t mode)
{
	struct net_device *netdev = (struct net_device *)app_data;
	struct nss_dp_dev *dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);
	int err;

	/* TODO: init the EDMA */

	if (dp_dev->drv_flags & NSS_DP_PRIV_FLAG(IRQ_REQUESTED))
		return NSS_DP_SUCCESS;

	err = request_irq(netdev->irq, edma_handle_irq, 0, "nss-dp",
						dp_dev);
	if (err) {
		netdev_dbg(netdev, "Mac %d IRQ %d request failed\n",
						dp_dev->macid, netdev->irq);
		return NSS_DP_FAILURE;
	}
	dp_dev->drv_flags |= NSS_DP_PRIV_FLAG(IRQ_REQUESTED);

	return NSS_DP_SUCCESS;
}

/*
 * edma_if_close()
 *	Do slow path data plane close
 */
static int edma_if_close(void *app_data)
{
	struct net_device *netdev = (struct net_device *)app_data;
	struct nss_dp_dev *dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);

	if (dp_dev->drv_flags & NSS_DP_PRIV_FLAG(IRQ_REQUESTED)) {
		netdev_dbg(netdev, "Freeing IRQ %d for Mac %d\n", netdev->irq,
								dp_dev->macid);
		free_irq(netdev->irq, dp_dev);
		dp_dev->drv_flags &= ~NSS_DP_PRIV_FLAG(IRQ_REQUESTED);
	}
	return NSS_DP_SUCCESS;
}

/*
 * edma_if_link_state()
 */
static int edma_if_link_state(void *app_data, uint32_t link_state)
{
	return NSS_DP_SUCCESS;
}

/*
 * edma_if_mac_addr()
 */
static int edma_if_mac_addr(void *app_data, uint8_t *addr)
{
	return NSS_DP_SUCCESS;
}

/*
 * edma_if_change_mtu()
 */
static int edma_if_change_mtu(void *app_data, uint32_t mtu)
{
	return NSS_DP_SUCCESS;
}

/*
 * edma_if_xmit()
 */
static int edma_if_xmit(void *app_data, struct sk_buff *skb)
{
	/*
	 * Fill-in descriptor
	 */

	return NSS_DP_FAILURE;
}

/*
 * edma_if_set_features()
 *	Set the supported net_device features
 */
static void edma_if_set_features(void *app_data)
{
	struct net_device *netdev = (struct net_device *)app_data;

	netdev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	netdev->hw_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	netdev->vlan_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	netdev->wanted_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
}

/*
 * edma_if_pause_on_off()
 *	Set pause frames on or off
 *
 * No need to send a message if we defaulted to slow path.
 */
static int edma_if_pause_on_off(void *app_data, uint32_t pause_on)
{
	return NSS_DP_SUCCESS;
}

/*
 * edma_if_init()
 */
static int edma_if_init(void *app_data)
{
	struct net_device *netdev = (struct net_device *)app_data;
	struct nss_dp_dev *dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);

	dp_dev->drv_flags |= NSS_DP_PRIV_FLAG(INIT_DONE);

	return NSS_DP_SUCCESS;
}

/*
 * nss_dp_edma_ops
 */
struct nss_dp_data_plane_ops nss_dp_edma_ops = {
	.init		= edma_if_init,
	.open		= edma_if_open,
	.close		= edma_if_close,
	.link_state	= edma_if_link_state,
	.mac_addr	= edma_if_mac_addr,
	.change_mtu	= edma_if_change_mtu,
	.xmit		= edma_if_xmit,
	.set_features	= edma_if_set_features,
	.pause_on_off	= edma_if_pause_on_off,
};
