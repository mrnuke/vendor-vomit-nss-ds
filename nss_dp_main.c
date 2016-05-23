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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/ethtool.h>

#include "nss_dp_dev.h"

/* Global data */
struct nss_dp_global_ctx ctx;

/*
 * nss_dp_do_ioctl()
 */
static int32_t nss_dp_do_ioctl(struct net_device *netdev, struct ifreq *ifr,
						   int32_t cmd)
{
	int ret = -EINVAL;
	struct nss_dp_dev *dp_dev;

	if (!netdev || !ifr)
		return ret;

	dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);

	/* TODO: Perform private ioctl operations */

	return 0;
}

/*
 * nss_dp_change_mtu()
 */
static int32_t nss_dp_change_mtu(struct net_device *netdev, int32_t newmtu)
{
	int ret = -EINVAL;
	struct nss_dp_dev *dp_dev;

	if (!netdev)
		return ret;

	dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);

	/* Let the underlying data plane decide if the newmtu is applicable */
	if (dp_dev->data_plane_ops->change_mtu(dp_dev->data_plane_ctx,
								newmtu)) {
		netdev_dbg(netdev, "Data plane change mtu failed\n");
		return ret;
	}

	netdev->mtu = newmtu;

	return 0;
}

/*
 * nss_dp_set_mac_address()
 */
static int32_t nss_dp_set_mac_address(struct net_device *netdev, void *macaddr)
{
	struct nss_dp_dev *dp_dev;
	struct sockaddr *addr = (struct sockaddr *)macaddr;
	int ret = -EINVAL;

	if (!netdev)
		return ret;

	dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);

	netdev_dbg(netdev, "AddrFamily: %d, %0x:%0x:%0x:%0x:%0x:%0x\n",
			addr->sa_family, addr->sa_data[0], addr->sa_data[1],
			addr->sa_data[2], addr->sa_data[3], addr->sa_data[4],
			addr->sa_data[5]);

	ret = eth_prepare_mac_addr_change(netdev, macaddr);
	if (ret)
		return ret;

	if (dp_dev->data_plane_ops->mac_addr(dp_dev->data_plane_ctx, macaddr)) {
		netdev_dbg(netdev, "Data plane set MAC address failed\n");
		return -EAGAIN;
	}

	eth_commit_mac_addr_change(netdev, macaddr);

	return 0;
}

/*
 * nss_dp_get_stats64()
 */
static struct rtnl_link_stats64 *nss_dp_get_stats64(struct net_device *netdev,
					     struct rtnl_link_stats64 *stats)
{
	struct nss_dp_dev *dp_dev;

	if (!netdev)
		return stats;

	dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);

	/* TODO: Call data plane fill stats */

	return stats;
}

/*
 * nss_dp_xmit()
 */
static netdev_tx_t nss_dp_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct nss_dp_dev *dp_dev;

	if (!skb || !netdev)
		return NETDEV_TX_OK;

	if (skb->len < ETH_HLEN) {
		netdev_dbg(netdev, "skb->len < ETH_HLEN\n");
		goto drop;
	}

	dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);

	netdev_dbg(netdev, "Tx packet, len %d\n", skb->len);

	if (likely(!dp_dev->data_plane_ops->xmit(dp_dev->data_plane_ctx, skb)))
		return NETDEV_TX_OK;

drop:
	netdev_dbg(netdev, "dropping skb\n");
	dev_kfree_skb_any(skb);
	netdev->stats.tx_dropped++;

	return NETDEV_TX_OK;
}

/*
 * nss_dp_close()
 */
static int nss_dp_close(struct net_device *netdev)
{
	struct nss_dp_dev *dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);

	if (!dp_dev)
		return -EINVAL;

	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	/* TODO: Call soc_hal_ops to stop GMAC tx/rx/intr */

	/* Notify data plane link is going down */
	if (dp_dev->data_plane_ops->link_state(dp_dev->data_plane_ctx, 0)) {
		netdev_dbg(netdev, "Data plane set link failed\n");
		return -EAGAIN;
	}

	/* TODO: Stop phy related stuff */

	/* Notify data plane to close */
	if (dp_dev->data_plane_ops->close(dp_dev->data_plane_ctx)) {
		netdev_dbg(netdev, "Data plane close failed\n");
		return -EAGAIN;
	}

	clear_bit(__NSS_DP_UP, &dp_dev->flags);

	return 0;
}

/*
 * nss_dp_open()
 */
static int nss_dp_open(struct net_device *netdev)
{
	struct nss_dp_dev *dp_dev = (struct nss_dp_dev *)netdev_priv(netdev);
	if (!dp_dev)
		return -EINVAL;

	netif_carrier_off(netdev);

	/*
	 * Call data plane init if it has not been done yet
	 */
	if (!(dp_dev->drv_flags & NSS_DP_PRIV_FLAG(INIT_DONE)))
		if (dp_dev->data_plane_ops->init(dp_dev->data_plane_ctx)) {
			netdev_dbg(netdev, "Data plane init failed\n");
			return -ENOMEM;
		}

	/*
	 * Inform the Linux Networking stack about the hardwar capability of
	 * checksum offloading and other features. Each data_plane is
	 * responsible to maintain the feature set it supports
	 */
	dp_dev->data_plane_ops->set_features(dp_dev->data_plane_ctx);

	set_bit(__NSS_DP_UP, &dp_dev->flags);
	netif_start_queue(netdev);

	/* TODO: Call soc_hal_ops to bring up GMAC */

	if (dp_dev->data_plane_ops->mac_addr(dp_dev->data_plane_ctx,
							netdev->dev_addr)) {
		netdev_dbg(netdev, "Data plane set MAC address failed\n");
		return -EAGAIN;
	}

	if (dp_dev->data_plane_ops->open(dp_dev->data_plane_ctx, 0, 0, 0)) {
		netdev_dbg(netdev, "Data plane open failed\n");
		return -EAGAIN;
	}

	netif_start_queue(netdev);
	netif_carrier_on(netdev);

	return 0;
}

/*
 * Netdevice operations
 */
static const struct net_device_ops nss_dp_netdev_ops = {
	.ndo_open = &nss_dp_open,
	.ndo_stop = &nss_dp_close,
	.ndo_start_xmit = &nss_dp_xmit,
	.ndo_get_stats64 = &nss_dp_get_stats64,
	.ndo_set_mac_address = &nss_dp_set_mac_address,
	.ndo_validate_addr = &eth_validate_addr,
	.ndo_change_mtu = &nss_dp_change_mtu,
	.ndo_do_ioctl = &nss_dp_do_ioctl,
};

/*
 * nss_dp_of_get_pdata()
 */
static int32_t nss_dp_of_get_pdata(struct device_node *np,
				   struct net_device *netdev)
{
	uint8_t *maddr;
	struct nss_dp_dev *dp_dev;

	dp_dev = netdev_priv(netdev);

	if (of_property_read_u32(np, "qcom,id", &dp_dev->macid)) {
		pr_err("%s: error reading critical device node properties\n",
						np->name);
		return -EFAULT;
	}

	dp_dev->phy_mii_type = of_get_phy_mode(np);

	/* TODO: Read IRQ...etc in */

	maddr = (uint8_t *)of_get_mac_address(np);
	if (maddr && is_valid_ether_addr(maddr)) {
		ether_addr_copy(netdev->dev_addr, maddr);
	} else {
		random_ether_addr(netdev->dev_addr);
		pr_info("GMAC%d(%p) Invalid MAC@ - using %pM\n", dp_dev->macid,
						dp_dev, netdev->dev_addr);
	}

	return 0;
}

/*
 * nss_dp_probe()
 */
static int32_t nss_dp_probe(struct platform_device *pdev)
{
	struct net_device *netdev;
	struct nss_dp_dev *dp_dev;
	struct device_node *np = pdev->dev.of_node;
	int32_t ret = 0;

	/* TODO: See if we need to do some SoC level common init */

	netdev = alloc_etherdev(sizeof(struct nss_dp_dev));
	if (!netdev) {
		pr_info("alloc_etherdev() failed\n");
		return -ENOMEM;
	}

	dp_dev = netdev_priv(netdev);
	memset((void *)dp_dev, 0, sizeof(struct nss_dp_dev));

	dp_dev->pdev = pdev;
	dp_dev->netdev = netdev;
	netdev->watchdog_timeo = 5 * HZ;
	netdev->netdev_ops = &nss_dp_netdev_ops;

	/* Use EDMA data plane as default */
	dp_dev->data_plane_ops = &nss_dp_edma_ops;
	dp_dev->data_plane_ctx = (void *)netdev;

	ret = nss_dp_of_get_pdata(np, netdev);
	if (ret != 0) {
		free_netdev(netdev);
		return ret;
	}

	dp_dev->ctx = &ctx;
	ctx.nss_dp[dp_dev->macid] = dp_dev;

	/* TODO:locks init */

	/* TODO: Call soc_hal_ops->init() to init individual GMACs */

	/* TODO: PHY related work */

	/* TODO: Features: CSUM, tx/rx offload... configure */

	/* Register the network interface */
	ret = register_netdev(netdev);
	if (ret) {
		netdev_dbg(netdev, "Error registering netdevice %s\n",
								netdev->name);
		free_netdev(netdev);
		return ret;
	}

	/* TODO: set ethtools */

	netdev_dbg(netdev, "Initialized NSS DP GMAC%d interface %s: (base = 0x%lx, irq = %d)\n",
		   dp_dev->macid, netdev->name, netdev->base_addr, netdev->irq);

	return 0;
}


/*
 * nss_dp_remove()
 */
static int nss_dp_remove(struct platform_device *pdev)
{
	uint32_t i;
	struct nss_dp_dev *dp_dev;

	for (i = 0; i < NSS_DP_MAX_PHY_PORTS; i++) {
		dp_dev = ctx.nss_dp[i];
		if (dp_dev) {
			/* TODO: Remove PHY */
			unregister_netdev(dp_dev->netdev);
			free_netdev(dp_dev->netdev);
			/* TODO: soc_hal_ops->remove() */
			ctx.nss_dp[i] = NULL;
		}
	}

	return 0;
}

static struct of_device_id nss_dp_dt_ids[] = {
	{ .compatible = "qcom,nss-dp" },
	{},
};
MODULE_DEVICE_TABLE(of, nss_dp_dt_ids);

static struct platform_driver nss_dp_drv = {
	.probe = nss_dp_probe,
	.remove = nss_dp_remove,
	.driver = {
		   .name = "nss-dp",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(nss_dp_dt_ids),
		  },
};

/*
 * nss_dp_init()
 */
int __init nss_dp_init(void)
{
	int ret;

	pr_info("**********************************************************\n");
	pr_info("* NSS Data Plane driver\n");
	pr_info("**********************************************************\n");

	/*
	 * Do the provisioning here, this is for bring up purpose, it should be
	 * done by SSDK
	 */
	rumi_test_init();

	ret = platform_driver_register(&nss_dp_drv);
	if (ret)
		pr_info("NSS DP platform drv register failed\n");

	return ret;
}

/*
 * nss_dp_exit()
 */
void __exit nss_dp_exit(void)
{
	platform_driver_unregister(&nss_dp_drv);
}

module_init(nss_dp_init);
module_exit(nss_dp_exit);

MODULE_AUTHOR("Qualcomm Atheros");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NSS Data Plane Network Driver");

