// SPDX-License-Identifier: GPL-2.0
/*
 * mctp-pcie-vdm.c - MCTP-over-PCIe-VDM (DMTF DSP0238) transport binding driver.
 *
 * DSP0238 is available at:
 * https://www.dmtf.org/sites/default/files/standards/documents/DSP0238_1.2.0.pdf
 *
 */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/dynamic_debug.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mctp-pcie-vdm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/ptr_ring.h>
#include <linux/skbuff.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/mctp.h>
#include <net/mctpdevice.h>

#define MCTP_PCIE_VDM_MIN_MTU 64
#define MCTP_PCIE_VDM_MAX_MTU 512
/* 16byte */
#define MCTP_PCIE_VDM_HDR_SIZE 16
#define MCTP_PAYLOAD_IC_TYPE_SIZE 1
#define MCTP_RECEIVE_PKT_TIMEOUT_MS 5

#define MCTP_PCIE_VDM_NET_DEV_TX_QUEUE_LEN 1100
#define MCTP_PCIE_VDM_DEV_TX_QUEUE_SIZE 64

#define MCTP_PCIE_VDM_FMT_4DW 0x3
#define MCTP_PCIE_VDM_TYPE_MSG 0x10
#define MCTP_PCIE_VDM_CODE 0x0
/* PCIe VDM message code */
#define MCTP_PCIE_VDM_MSG_CODE 0x7F
#define MCTP_PCIE_VDM_VENDOR_ID 0x1AB4
/* MCTP message type */
#define MCTP_MSG_TYPE_MASK GENMASK(6, 0)
#define MCTP_PCIE_VDM_MSG_TYPE 0x7E

#define MCTP_PCIE_SWAP_NET_ENDIAN(arr, len)       \
	do {                                      \
		u32 *p = (u32 *)(arr);            \
		for (int i = 0; i < (len); i++) { \
			p[i] = htonl(p[i]);       \
		}                                 \
	} while (0)

#define MCTP_PCIE_SWAP_LITTLE_ENDIAN(arr, len)      \
	do {                                      \
		u32 *p = (u32 *)(arr);            \
		for (int i = 0; i < (len); i++) { \
			p[i] = ntohl(p[i]);       \
			p[i] = cpu_to_le32(p[i]); \
		}                                 \
	} while (0)

enum mctp_pcie_vdm_route_type {
	MCTP_PCIE_VDM_ROUTE_TO_RC = 0,
	MCTP_PCIE_VDM_ROUTE_BY_ID = 2,
	MCTP_PCIE_VDM_BROADCAST_FROM_RC = 3,
};

enum mctp_ctrl_command_code {
	MCTP_CTRL_CMD_SET_ENDPOINT_ID = 0x01,
	MCTP_CTRL_CMD_GET_ENDPOINT_ID = 0x02,
	MCTP_CTRL_CMD_PREPARE_ENDPOINT_DISCOVERY = 0x0B,
	MCTP_CTRL_CMD_ENDPOINT_DISCOVERY = 0x0C,
	MCTP_CTRL_CMD_DISCOVERY_NOTIFY = 0x0D
};

struct mctp_pcie_vdm_hdr {
	u32 length : 10, rsvd0 : 2, attr : 2, ep : 1, td : 1, rsvd1 : 4, tc : 3,
		rsvd2 : 1, route_type : 5, fmt : 2, rsvd3 : 1;
	u8 msg_code;
	u8 tag_vdm_code : 4, tag_pad_len : 2, tag_rsvd : 2;
	u16 pci_req_id;
	u16 pci_vendor_id;
	u16 pci_target_id;
};

struct mctp_pcie_vdm_dev {
	struct device *dev;
	const struct mctp_pcie_vdm_ops *callback_ops;
};


static const struct mctp_pcie_vdm_hdr mctp_pcie_vdm_hdr_template = {
	.fmt = MCTP_PCIE_VDM_FMT_4DW,
	.route_type = MCTP_PCIE_VDM_TYPE_MSG | MCTP_PCIE_VDM_ROUTE_BY_ID,
	.tag_vdm_code = MCTP_PCIE_VDM_CODE,
	.msg_code = MCTP_PCIE_VDM_MSG_CODE,
	.pci_vendor_id = MCTP_PCIE_VDM_VENDOR_ID,
	.attr = 0,
};

static void mctp_pcie_vdm_display_skb_buff_data(struct sk_buff *skb)
{
	int i = 0;

	while ((i + 4) < skb->len) {
		pr_debug("%02x %02x %02x %02x\n", skb->data[i],
			 skb->data[i + 1], skb->data[i + 2], skb->data[i + 3]);
		i += 4;
	}

	char buf[16] = { 0 };
	char *p = buf;

	while (i < skb->len) {
		p += snprintf(p, sizeof(buf) - (p - buf), "%02x ",
			      skb->data[i]);
		i++;
	}
	pr_debug("%s\n", buf);
}

static int mctp_pcie_vdm_xmit(struct net_device *ndev, struct sk_buff *skb)
{
	struct net_device_stats *stats;
	struct mctp_pcie_vdm_hdr *hdr;
	struct mctp_pcie_vdm_dev *vdm_dev;
	u8 *hdr_byte;
	u16 payload_len_dw;
	u16 payload_len_byte;
	int rc;

	stats = &ndev->stats;
	vdm_dev = netdev_priv(ndev);
	hdr = (struct mctp_pcie_vdm_hdr *)skb->data;
	hdr_byte = skb->data;
	payload_len_dw = (ALIGN(skb->len, sizeof(u32)) - MCTP_PCIE_VDM_HDR_SIZE) / sizeof(u32);
	payload_len_byte = skb->len - MCTP_PCIE_VDM_HDR_SIZE;

	hdr->length = payload_len_dw;
	hdr->tag_pad_len =
		ALIGN(payload_len_byte, sizeof(u32)) - payload_len_byte;
	pr_debug("%s: skb len %d pad len %d\n", __func__, skb->len,
		 hdr->tag_pad_len);
	MCTP_PCIE_SWAP_NET_ENDIAN((u32 *)hdr,
				  sizeof(struct mctp_pcie_vdm_hdr) / sizeof(u32));

	mctp_pcie_vdm_display_skb_buff_data(skb);
	rc = vdm_dev->callback_ops->send_packet(vdm_dev->dev, skb->data, payload_len_dw * sizeof(u32));

	if (rc) {
		pr_err("%s: failed to send packet, rc %d\n", __func__, rc);
		stats->tx_errors++;
	} else {
		stats->tx_packets++;
		stats->tx_bytes += (skb->len - sizeof(struct mctp_pcie_vdm_hdr));
	}
	return rc;
}

static netdev_tx_t mctp_pcie_vdm_start_xmit(struct sk_buff *skb,
					    struct net_device *ndev)
{
	int rc;
	netdev_tx_t ret;

	pr_debug("%s: skb len %u\n", __func__, skb->len);

	if (skb) {
		rc = mctp_pcie_vdm_xmit(ndev, skb);
		if (rc) {
			pr_err("%s: failed to send packet, rc %d\n", __func__, rc);
			ret = NETDEV_TX_BUSY;
		} else {
			ret = NETDEV_TX_OK;
			kfree_skb(skb);
		}
	}
	return ret;
}

static void mctp_pcie_vdm_uninit(struct net_device *ndev)
{
	struct mctp_pcie_vdm_dev *vdm_dev;

	vdm_dev = netdev_priv(ndev);
	pr_info("%s: uninitializing vdm_dev %s\n", __func__,
		ndev->name);
	vdm_dev->callback_ops->uninit(vdm_dev->dev);
}

static int mctp_pcie_vdm_hdr_create(struct sk_buff *skb,
				    struct net_device *ndev,
				    unsigned short type, const void *daddr,
				    const void *saddr, unsigned int len)
{
	u8 dest_addr[3] = {0};
	struct mctp_pcie_vdm_hdr *hdr =
		(struct mctp_pcie_vdm_hdr *)skb_push(skb, sizeof(*hdr));

	pr_debug("%s type %d len %d\n", __func__, type, len);
	memcpy(hdr, &mctp_pcie_vdm_hdr_template, sizeof(*hdr));
	if (daddr) {
		memcpy(dest_addr, (u8 *)daddr, sizeof(dest_addr));
		hdr->route_type |= dest_addr[0] & GENMASK(2, 0);
		hdr->pci_target_id = dest_addr[1] << 8 | dest_addr[2];
		pr_debug("%s dst route %d addr %d\n", __func__, hdr->route_type, hdr->pci_target_id);
	}

	if (saddr) {
		pr_debug("%s src addr %d\n", __func__, *(u16 *)saddr);
		hdr->pci_req_id = *(u16 *)saddr;
	}

	return 0;
}

static const struct net_device_ops mctp_pcie_vdm_net_ops = {
	.ndo_start_xmit = mctp_pcie_vdm_start_xmit,
	.ndo_uninit = mctp_pcie_vdm_uninit,
};

static const struct header_ops mctp_pcie_vdm_net_hdr_ops = {
	.create = mctp_pcie_vdm_hdr_create,
};

static void mctp_pcie_vdm_net_setup(struct net_device *ndev)
{
	ndev->type = ARPHRD_MCTP;

	ndev->mtu = MCTP_PCIE_VDM_MIN_MTU;
	ndev->min_mtu = MCTP_PCIE_VDM_MIN_MTU;
	ndev->max_mtu = MCTP_PCIE_VDM_MAX_MTU;
	ndev->tx_queue_len = MCTP_PCIE_VDM_NET_DEV_TX_QUEUE_LEN;
	ndev->addr_len = 3; //PCIe bdf is 2bytes + 1byte route type
	ndev->hard_header_len = sizeof(struct mctp_pcie_vdm_hdr);

	ndev->netdev_ops = &mctp_pcie_vdm_net_ops;
	ndev->header_ops = &mctp_pcie_vdm_net_hdr_ops;
}

static int mctp_pcie_vdm_add_net_dev(struct net_device **dev)
{
	struct net_device *ndev = alloc_netdev(sizeof(struct mctp_pcie_vdm_dev),
					       "mctppci%d", NET_NAME_UNKNOWN,
					       mctp_pcie_vdm_net_setup);

	if (!ndev) {
		pr_err("%s: failed to allocate net device\n", __func__);
		return -ENOMEM;
	}
	dev_net_set(ndev, current->nsproxy->net_ns);

	*dev = ndev;
	int rc;

	rc = mctp_register_netdev(ndev, NULL, MCTP_PHYS_BINDING_PCIE_VDM);
	if (rc) {
		pr_err("%s: failed to register net device\n", __func__);
		free_netdev(ndev);
		return rc;
	}
	return rc;
}

void mctp_pcie_vdm_receive_packet(struct net_device *ndev)
{
	struct mctp_pcie_vdm_dev *vdm_dev;
	u8 *packet;

	vdm_dev = netdev_priv(ndev);
	packet = vdm_dev->callback_ops->recv_packet(vdm_dev->dev);

	while (!IS_ERR(packet)) {
		MCTP_PCIE_SWAP_LITTLE_ENDIAN((u32 *)packet,
					     sizeof(struct mctp_pcie_vdm_hdr) / sizeof(u32));
		struct mctp_pcie_vdm_hdr *vdm_hdr = (struct mctp_pcie_vdm_hdr *)packet;
		struct mctp_skb_cb *cb;
		struct net_device_stats *stats;
		struct sk_buff *skb;
		u16 len;
		int net_status;

		stats = &ndev->stats;
		len = vdm_hdr->length * sizeof(u32) -
				vdm_hdr->tag_pad_len;
		len += (MCTP_PCIE_VDM_HDR_SIZE - sizeof(struct mctp_pcie_vdm_hdr));
		skb = netdev_alloc_skb(ndev, len);
		pr_debug("%s: received packet size: %d\n", __func__,
			 len);

		if (!skb) {
			stats->rx_errors++;
			pr_err("%s: failed to alloc skb\n", __func__);
			continue;
		}

		skb->protocol = htons(ETH_P_MCTP);
		/* put data into tail sk buff */
		skb_put_data(skb, &packet[sizeof(struct mctp_pcie_vdm_hdr)], len);
		mctp_pcie_vdm_display_skb_buff_data(skb);

		cb = __mctp_cb(skb);
		cb->halen = 3; // route type | bdf address
		cb->haddr[0] = vdm_hdr->route_type;
		// address is also converted to little-endian, but we want to keep it as big-endian
		// because kernel network layer assumes address in big-endian format
		cb->haddr[1] = vdm_hdr->pci_req_id & 0xFF;
		cb->haddr[2] = vdm_hdr->pci_req_id >> 8;

		net_status = netif_rx(skb);
		if (net_status == NET_RX_SUCCESS) {
			stats->rx_packets++;
			stats->rx_bytes += len;
		} else {
			stats->rx_dropped++;
		}

		vdm_dev->callback_ops->free_packet(packet);
		packet = vdm_dev->callback_ops->recv_packet(vdm_dev->dev);
	}
}

struct net_device *mctp_pcie_vdm_add_dev(struct device *dev,
					 const struct mctp_pcie_vdm_ops *ops)
{
	struct net_device *ndev;
	struct mctp_pcie_vdm_dev *vdm_dev;
	int rc;

	rc = mctp_pcie_vdm_add_net_dev(&ndev);
	if (rc) {
		pr_err("%s: failed to add net device\n", __func__);
		return ERR_PTR(rc);
	}

	vdm_dev = netdev_priv(ndev);
	vdm_dev->dev = dev;
	vdm_dev->callback_ops = ops;

	return ndev;
}
EXPORT_SYMBOL_GPL(mctp_pcie_vdm_add_dev);

void mctp_pcie_vdm_remove_dev(struct net_device *vdm_dev)
{
	pr_debug("%s: removing vdm_dev %s\n", __func__, vdm_dev->name);

	if (vdm_dev) {
		mctp_unregister_netdev(vdm_dev);
		free_netdev(vdm_dev);
	}
}
EXPORT_SYMBOL_GPL(mctp_pcie_vdm_remove_dev);
