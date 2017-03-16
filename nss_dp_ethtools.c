/*
 **************************************************************************
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#include <linux/ethtool.h>
#include "nss_dp_dev.h"

/*
 * nss_dp_get_ethtool_stats()
 */
static void nss_dp_get_ethtool_stats(struct net_device *netdev,
				struct ethtool_stats *stats, uint64_t *data)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(netdev);

	dp_priv->gmac_hal_ops->getethtoolstats(dp_priv->gmac_hal_ctx, data);
}

/*
 * nss_dp_get_strset_count()
 */
static int32_t nss_dp_get_strset_count(struct net_device *netdev, int32_t sset)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(netdev);

	return dp_priv->gmac_hal_ops->getssetcount(dp_priv->gmac_hal_ctx, sset);
}

/*
 * nss_dp_get_strings()
 */
static void nss_dp_get_strings(struct net_device *netdev, uint32_t stringset,
			uint8_t *data)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(netdev);

	dp_priv->gmac_hal_ops->getstrings(dp_priv->gmac_hal_ctx, stringset,
					  data);
}

/*
 * nss_dp_get_settings()
 */
static int32_t nss_dp_get_settings(struct net_device *netdev,
				   struct ethtool_cmd *cmd)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(netdev);
	uint32_t speed;

	/*
	 * If there is a PHY attached, get the status from Kernel helper
	 */
	if (dp_priv->phydev)
		return phy_ethtool_gset(dp_priv->phydev, cmd);

	speed = dp_priv->gmac_hal_ops->getspeed(dp_priv->gmac_hal_ctx);
	ethtool_cmd_speed_set(cmd, speed);
	cmd->duplex = dp_priv->gmac_hal_ops->getduplex(dp_priv->gmac_hal_ctx);

	return 0;
}

/*
 * nss_dp_set_settings()
 */
static int32_t nss_dp_set_settings(struct net_device *netdev,
				  struct ethtool_cmd *cmd)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(netdev);

	if (!dp_priv->phydev)
		return -EIO;

	return phy_ethtool_sset(dp_priv->phydev, cmd);
}

/*
 * nss_dp_get_pauseparam()
 */
static void nss_dp_get_pauseparam(struct net_device *netdev,
				     struct ethtool_pauseparam *pause)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(netdev);

	pause->rx_pause = dp_priv->pause & FLOW_CTRL_RX ? 1 : 0;
	pause->tx_pause = dp_priv->pause & FLOW_CTRL_TX ? 1 : 0;
	pause->autoneg = AUTONEG_ENABLE;
}

/*
 * nss_dp_set_pauseparam()
 */
static int32_t nss_dp_set_pauseparam(struct net_device *netdev,
				     struct ethtool_pauseparam *pause)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(netdev);

	/* set flow control settings */
	dp_priv->pause = 0;
	if (pause->rx_pause)
		dp_priv->pause |= FLOW_CTRL_RX;

	if (pause->tx_pause)
		dp_priv->pause |= FLOW_CTRL_TX;

	if (!dp_priv->phydev)
		return 0;

	/* Update flow control advertisment */
	dp_priv->phydev->advertising &=
				~(ADVERTISED_Pause | ADVERTISED_Asym_Pause);

	if (pause->rx_pause)
		dp_priv->phydev->advertising |=
				(ADVERTISED_Pause | ADVERTISED_Asym_Pause);

	if (pause->tx_pause)
		dp_priv->phydev->advertising |= ADVERTISED_Asym_Pause;

	genphy_config_aneg(dp_priv->phydev);

	return 0;
}

/*
 * Ethtool operations
 */
struct ethtool_ops nss_dp_ethtool_ops = {
	.get_strings = &nss_dp_get_strings,
	.get_sset_count = &nss_dp_get_strset_count,
	.get_ethtool_stats = &nss_dp_get_ethtool_stats,
	.get_link = &ethtool_op_get_link,
	.get_settings = &nss_dp_get_settings,
	.set_settings = &nss_dp_set_settings,
	.get_pauseparam = &nss_dp_get_pauseparam,
	.set_pauseparam = &nss_dp_set_pauseparam,
};

/*
 * nss_dp_set_ethtool_ops()
 *	Set ethtool operations
 */
void nss_dp_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &nss_dp_ethtool_ops;
}
