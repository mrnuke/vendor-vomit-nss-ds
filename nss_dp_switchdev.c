/*
 **************************************************************************
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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

#include <net/switchdev.h>
#include <linux/if_bridge.h>
#include "nss_dp_dev.h"
#include "fal/fal_stp.h"

#define NSS_DP_SWITCH_ID		0

/*
 * nss_dp_attr_get()
 *	Get port information to update switchdev attribute for NSS data plane.
 */
static int nss_dp_attr_get(struct net_device *dev, struct switchdev_attr *attr)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(dev);

	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_PARENT_ID:
		attr->u.ppid.id_len = 1;
		attr->u.ppid.id[0] = NSS_DP_SWITCH_ID;
		break;
	case SWITCHDEV_ATTR_ID_PORT_BRIDGE_FLAGS:
		attr->u.brport_flags = dp_priv->brport_flags;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/*
 * nss_dp_stp_state_set()
 *	Set bridge port STP state to the port of NSS data plane.
 */
static int nss_dp_stp_state_set(struct nss_dp_dev *dp_priv, u8 state)
{
	sw_error_t err;
	fal_stp_state_t stp_state;

	switch (state) {
	case BR_STATE_DISABLED:
		stp_state = FAL_STP_DISABLED;
		break;
	case BR_STATE_LISTENING:
		stp_state = FAL_STP_LISTENING;
		break;
	case BR_STATE_BLOCKING:
		stp_state = FAL_STP_BLOCKING;
		break;
	case BR_STATE_LEARNING:
		stp_state = FAL_STP_LEARNING;
		break;
	case BR_STATE_FORWARDING:
		stp_state = FAL_STP_FORWARDING;
		break;
	default:
		return -EOPNOTSUPP;
	}

	err = fal_stp_port_state_set(NSS_DP_SWITCH_ID, 0, dp_priv->macid,
				     stp_state);
	if (err) {
		netdev_dbg(dp_priv->netdev, "failed to set ftp state\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * nss_dp_attr_set()
 *	Get switchdev attribute and set to the device of NSS data plane.
 */
static int nss_dp_attr_set(struct net_device *dev,
				const struct switchdev_attr *attr,
				struct switchdev_trans *trans)
{
	struct nss_dp_dev *dp_priv = (struct nss_dp_dev *)netdev_priv(dev);

	if (switchdev_trans_ph_prepare(trans))
		return 0;

	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_BRIDGE_FLAGS:
		dp_priv->brport_flags = attr->u.brport_flags;
		netdev_dbg(dev, "set brport_flags %lu\n", attr->u.brport_flags);
		return 0;
	case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
		return nss_dp_stp_state_set(dp_priv, attr->u.stp_state);
	default:
		return -EOPNOTSUPP;
	}
}

/*
 * nss_dp_switchdev_ops
 *	Switchdev operations of NSS data plane.
 */
static const struct switchdev_ops nss_dp_switchdev_ops = {
	.switchdev_port_attr_get	= nss_dp_attr_get,
	.switchdev_port_attr_set	= nss_dp_attr_set,
};

/*
 * nss_dp_switchdev_setup()
 *	Set up NSS data plane switchdev operations.
 */
void nss_dp_switchdev_setup(struct net_device *dev)
{
#ifdef CONFIG_NET_SWITCHDEV
	dev->switchdev_ops = &nss_dp_switchdev_ops;
#endif
	switchdev_port_fwd_mark_set(dev, NULL, false);
}
