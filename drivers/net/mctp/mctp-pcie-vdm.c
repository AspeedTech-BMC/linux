// SPDX-License-Identifier: GPL-2.0
/*
 * Implements DMTF specification
 * "DSP0238 Management Component Transport Protocol (MCTP) PCIe VDM Transport
 * Binding"
 *  https://www.dmtf.org/sites/default/files/standards/documents/DSP0238_1.2.0.pdf
 *
 * Copyright (c) 2023 Code Construct
 */

#include "linux/dynamic_debug.h"
#include "linux/if_ether.h"
#include "linux/mutex.h"
#include "linux/pci.h"
#include "linux/printk.h"
#include "linux/skbuff.h"
#include "linux/stddef.h"
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/ptr_ring.h>
#include <linux/bitops.h>
#include <linux/wait.h>
#include <linux/if_arp.h>
#include <linux/byteorder/generic.h>
#include <linux/aspeed-mctp.h>
#include <net/mctp.h>
#include <net/mctpdevice.h>

#define LEN_MASK_HI GENMASK(9, 8)
#define LEN_MASK_LO GENMASK(7, 0)
#define PCI_VDM_HDR_LEN_MASK_LO GENMASK(31, 24)
#define PCI_VDM_HDR_LEN_MASK_HI GENMASK(17, 16)
#define PCIE_VDM_HDR_REQUESTER_BDF_MASK GENMASK(31, 16)

/* 64byte MCTP payload + 16 byte PCIe binding header */
#define MCTP_PCIE_VDM_MIN_MTU (64 + 16)
#define MCTP_PCIE_VDM_MAX_MTU 512
/* 16byte */
#define MCTP_PCIE_VDM_HDR_SIZE 16
#define MCTP_PAYLOAD_IC_TYPE_SIZE 1

#define MCTP_PCIE_VDM_TX_QUEUE_LEN 1100

#define MCTP_PCIE_VDM_FMT_4DW 0x3
#define MCTP_PCIE_VDM_TYPE_MSG 0x10
#define MCTP_PCIE_VDM_CODE 0x0
/* PCIe VDM message code */
#define MCTP_PCIE_VDM_MSG_CODE 0x7F
#define MCTP_PCIE_VDM_VENDOR_ID 0x1AB4
/* MCTP message type */
#define MCTP_PCIE_VMD_MSG_TYPE 0x7E
#define MCTP_CONTROL_MSG_TYPE 0x00

#define MCTP_CTRL_MSG_RQDI_REQ 0x80
#define MCTP_CTRL_MSG_RQDI_RSP 0x00

#define MCTP_PCIE_SWAP_NET_ENDIAN(arr, len)       \
	do {                                      \
		u32 *p = (u32 *)(arr);            \
		for (int i = 0; i < (len); i++) { \
			p[i] = htonl(p[i]);       \
		}                                 \
	} while (0)

#define MCTP_PCIE_SWAP_HOST_ENDIAN(arr, len)      \
	do {                                      \
		u32 *p = (u32 *)(arr);            \
		for (int i = 0; i < (len); i++) { \
			p[i] = ntohl(p[i]);       \
		}                                 \
	} while (0)

enum mctp_pcie_vdm_route_type {
	MCTP_PCIE_VDM_ROUTE_TO_RC = 0,
	MCTP_PCIE_VDM_ROUTE_BY_ID = 2,
	MCTP_PCIE_VDM_BROADCAST_FROM_RC = 3,
};

enum mctp_ctrl_command_code {
	MCTP_CTRL_CMD_PREPARE_ENDPOINT_DISCOVERY = 0x0B,
	MCTP_CTRL_CMD_ENDPOINT_DISCOVERY = 0x0C,
	MCTP_CTRL_CMD_DISCOVERY_NOTIFY = 0x0D
};

struct mctp_ctrl_msg_hdr {
	u8 rq_dgram_inst;
	u8 command_code;
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
	struct net_device *ndev;
	struct task_struct *rx_thread;
	struct task_struct *tx_thread;
	struct mctp_client *client;
	struct sk_buff *tx_skb;
	/* lock tx_skb for tx thread to avoid race */
	spinlock_t tx_lock;
	/* lock receive_data for rx thread to avoid race */
	spinlock_t rx_lock;
	wait_queue_head_t tx_wait;
	wait_queue_head_t rx_wait;
	bool receive_data;
	struct list_head list;
};

/* mutex for vdm_devs add/delete */
DEFINE_MUTEX(mctp_pcie_vdm_dev_mutex);
LIST_HEAD(mctp_pcie_vdm_devs);

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

static netdev_tx_t mctp_pcie_vdm_start_xmit(struct sk_buff *skb,
					    struct net_device *ndev)
{
	struct mctp_pcie_vdm_dev *vdm_dev = netdev_priv(ndev);
	unsigned long flags;

	pr_debug("%s: skb len %u\n", __func__, skb->len);

	netdev_tx_t ret;

	netif_stop_queue(ndev);
	spin_lock_irqsave(&vdm_dev->tx_lock, flags);

	if (vdm_dev->tx_skb) {
		pr_err("%s: failed to send packet, handling previous\n",
		       __func__);
		ret = NETDEV_TX_BUSY;
	} else {
		vdm_dev->tx_skb = skb;
		ret = NETDEV_TX_OK;
	}
	spin_unlock_irqrestore(&vdm_dev->tx_lock, flags);

	if (ret == NETDEV_TX_OK)
		wake_up(&vdm_dev->tx_wait);

	return ret;
}

static void mctp_pcie_vdm_xmit(struct mctp_pcie_vdm_dev *vdm_dev,
			       struct sk_buff *skb)
{
	struct net_device_stats *stats = &vdm_dev->ndev->stats;
	struct mctp_pcie_vdm_hdr *hdr = (struct mctp_pcie_vdm_hdr *)skb->data;

	u8 *hdr_byte = (u8 *)hdr;
	u8 message_type = hdr_byte[MCTP_PCIE_VDM_HDR_SIZE];
	u16 payload_len_dw =
		(ALIGN(skb->len, sizeof(u32)) - MCTP_PCIE_VDM_HDR_SIZE) /
		sizeof(u32);
	struct mctp_ctrl_msg_hdr *ctrl_hdr =
		(struct mctp_ctrl_msg_hdr
			 *)(&hdr_byte[MCTP_PCIE_VDM_HDR_SIZE + 1]);

	/* mctp control request message  */
	if (message_type == MCTP_CONTROL_MSG_TYPE) {
		switch (ctrl_hdr->command_code) {
		case MCTP_CTRL_CMD_DISCOVERY_NOTIFY:
			hdr->route_type = MCTP_PCIE_VDM_TYPE_MSG |
					  MCTP_PCIE_VDM_ROUTE_TO_RC;
			hdr->pci_target_id = 0x0000;
			break;
		case MCTP_CTRL_CMD_PREPARE_ENDPOINT_DISCOVERY:
		case MCTP_CTRL_CMD_ENDPOINT_DISCOVERY:
			if (ctrl_hdr->rq_dgram_inst & MCTP_CTRL_MSG_RQDI_REQ) {
				hdr->route_type =
					MCTP_PCIE_VDM_TYPE_MSG |
					MCTP_PCIE_VDM_BROADCAST_FROM_RC;
				hdr->pci_target_id = 0xFFFF;
			} else if (ctrl_hdr->rq_dgram_inst ==
				   MCTP_CTRL_MSG_RQDI_RSP) {
				hdr->route_type = MCTP_PCIE_VDM_TYPE_MSG |
						  MCTP_PCIE_VDM_ROUTE_TO_RC;
			}
			break;
		default:
			/* Unknown command code */
			break;
		}
	}

	hdr->length = payload_len_dw;

	u16 payload_data_len = skb->len - MCTP_PCIE_VDM_HDR_SIZE;

	hdr->tag_pad_len =
		ALIGN(payload_data_len, sizeof(u32)) - payload_data_len;
	pr_debug("%s: skb len %d pad len %d\n", __func__, skb->len,
		 hdr->tag_pad_len);
	u16 len = (payload_len_dw * sizeof(uint32_t)) + MCTP_PCIE_VDM_HDR_SIZE;

	// freed at aspeed-mctp tx tasklet
	struct mctp_pcie_packet *packet = aspeed_mctp_packet_alloc(GFP_KERNEL);

	if (!packet) {
		pr_err("%s: failed to alloc packet\n", __func__);
		stats->tx_errors++;
		return;
	}

	memcpy(&packet->data.hdr, skb->data, MCTP_PCIE_VDM_HDR_SIZE);
	MCTP_PCIE_SWAP_NET_ENDIAN(packet->data.hdr,
				  sizeof(struct mctp_pcie_vdm_hdr) /
					  sizeof(u32));

	memcpy((u8 *)&packet->data.payload, &hdr_byte[MCTP_PCIE_VDM_HDR_SIZE],
	       len - MCTP_PCIE_VDM_HDR_SIZE);
	packet->size = len;
	pr_debug("%s: skb len %u, pkt len %u\n", __func__, skb->len,
		 packet->size);

	mctp_pcie_vdm_display_skb_buff_data(skb);

	int rc = aspeed_mctp_send_packet(vdm_dev->client, packet);

	if (rc) {
		pr_err("%s: failed to send packet, rc %d\n", __func__, rc);
		stats->tx_errors++;
		aspeed_mctp_packet_free(packet);
		return;
	}
	stats->tx_packets++;
	stats->tx_bytes += (skb->len - sizeof(struct mctp_pcie_vdm_hdr));
}

static int mctp_pcie_vdm_tx_thread(void *data)
{
	struct mctp_pcie_vdm_dev *vdm_dev = data;
	struct sk_buff *skb;
	unsigned long flags;

	for (;;) {
		if (kthread_should_stop())
			break;

		spin_lock_irqsave(&vdm_dev->tx_lock, flags);
		skb = vdm_dev->tx_skb;
		vdm_dev->tx_skb = NULL;
		spin_unlock_irqrestore(&vdm_dev->tx_lock, flags);

		if (netif_queue_stopped(vdm_dev->ndev))
			netif_wake_queue(vdm_dev->ndev);

		if (skb) {
			mctp_pcie_vdm_xmit(vdm_dev, skb);
			kfree_skb(skb);
		} else {
			wait_event_idle(vdm_dev->tx_wait,
					vdm_dev->tx_skb ||
						kthread_should_stop());
		}
	}

	pr_debug("%s stopping\n", __func__);
	return 0;
}

static int mctp_pcie_vdm_rx_thread(void *data)
{
	struct mctp_pcie_vdm_dev *vdm_dev = data;
	struct mctp_client *client = vdm_dev->client;
	unsigned long flags;

	while (!kthread_should_stop()) {
		struct mctp_pcie_packet *packet;

		wait_event_idle(vdm_dev->rx_wait, vdm_dev->receive_data);

		spin_lock_irqsave(&vdm_dev->rx_lock, flags);
		vdm_dev->receive_data = false;
		spin_unlock_irqrestore(&vdm_dev->rx_lock, flags);

		packet = aspeed_mctp_receive_packet(client,
						    msecs_to_jiffies(5000));
		if (IS_ERR(packet)) {
			if (PTR_ERR(packet) == -ETIME) {
				// No packet available, continue waiting
				continue;
			}
			pr_err("%s: recv packet failed, return error code: %ld\n",
			       __func__, PTR_ERR(packet));
			return PTR_ERR(packet);
		}

		MCTP_PCIE_SWAP_HOST_ENDIAN(packet->data.hdr,
					   sizeof(struct mctp_pcie_vdm_hdr) /
						   sizeof(u32));
		struct mctp_pcie_vdm_hdr *vdm_hdr =
			(struct mctp_pcie_vdm_hdr *)(&packet->data.hdr[0]);
		struct mctp_skb_cb *cb;
		struct net_device_stats *stats;
		struct sk_buff *skb;
		u16 len;
		int net_status;

		stats = &vdm_dev->ndev->stats;
		len = vdm_hdr->length * sizeof(u32) - vdm_hdr->tag_pad_len;
		len += MCTP_PCIE_VDM_HDR_SIZE;
		skb = netdev_alloc_skb(vdm_dev->ndev, len);
		pr_debug("%s: received packet size: %d\n", __func__, len);

		if (!skb) {
			stats->rx_errors++;
			pr_err("%s: failed to alloc skb\n", __func__);
			continue;
		}

		skb->protocol = htons(ETH_P_MCTP);
		/* put data into tail sk buff */
		skb_put_data(skb, (u8 *)&packet->data, len);
		/* remove first 12bytes PCIe VDM header */
		skb_pull(skb, sizeof(struct mctp_pcie_vdm_hdr));
		pr_debug("%s: skb len: %u\n", __func__, skb->len);
		mctp_pcie_vdm_display_skb_buff_data(skb);

		cb = __mctp_cb(skb);
		cb->halen = 2; // BDF size is 2 bytes
		memcpy(cb->haddr, &vdm_hdr->pci_req_id, cb->halen);

		net_status = netif_rx(skb);
		if (net_status == NET_RX_SUCCESS) {
			stats->rx_packets++;
			stats->rx_bytes += skb->len;
		} else {
			stats->rx_dropped++;
		}

		aspeed_mctp_packet_free(packet);
	}
	pr_debug("%s stopping\n", __func__);
	return 0;
}

static int mctp_pcie_vdm_add_mctp_dev(struct mctp_pcie_vdm_dev *vdm_dev,
				      struct aspeed_mctp *priv)
{
	struct mctp_client *mctp_client = aspeed_mctp_create_client(priv);

	if (!mctp_client) {
		pr_err("%s: failed to create mctp client\n", __func__);
		return -ENOMEM;
	}

	vdm_dev->client = mctp_client;
	vdm_dev->tx_skb = NULL;
	vdm_dev->receive_data = false;
	spin_lock_init(&vdm_dev->tx_lock);
	spin_lock_init(&vdm_dev->rx_lock);
	init_waitqueue_head(&vdm_dev->tx_wait);
	init_waitqueue_head(&vdm_dev->rx_wait);
	vdm_dev->rx_thread = kthread_run(mctp_pcie_vdm_rx_thread, vdm_dev,
					 "mctp_pcie_vdm_rx_thread");
	vdm_dev->tx_thread = kthread_run(mctp_pcie_vdm_tx_thread, vdm_dev,
					 "mctp_pcie_vdm_tx_thread");

	mutex_lock(&mctp_pcie_vdm_dev_mutex);
	list_add_tail(&vdm_dev->list, &mctp_pcie_vdm_devs);
	mutex_unlock(&mctp_pcie_vdm_dev_mutex);

	aspeed_mctp_register_default_handler(mctp_client);
	return 0;
}

static void mctp_pcie_vdm_uninit(struct net_device *ndev)
{
	struct mctp_pcie_vdm_dev *vdm_dev = netdev_priv(ndev);
	struct mctp_client *client = vdm_dev->client;

	if (client) {
		aspeed_mctp_flush_rx_queue(client);
		aspeed_mctp_delete_client(client);
		vdm_dev->client = NULL;
	}

	if (vdm_dev->rx_thread) {
		kthread_stop(vdm_dev->rx_thread);
		vdm_dev->rx_thread = NULL;
	}

	if (vdm_dev->tx_thread) {
		kthread_stop(vdm_dev->tx_thread);
		vdm_dev->tx_thread = NULL;
	}
}

static int mctp_pcie_vdm_hdr_create(struct sk_buff *skb,
				    struct net_device *ndev,
				    unsigned short type, const void *daddr,
				    const void *saddr, unsigned int len)
{
	struct mctp_pcie_vdm_hdr *hdr =
		(struct mctp_pcie_vdm_hdr *)skb_push(skb, sizeof(*hdr));

	pr_debug("%s type %d len %d\n", __func__, type, len);
	memcpy(hdr, &mctp_pcie_vdm_hdr_template, sizeof(*hdr));
	if (daddr) {
		pr_debug("%s dst addr %d\n", __func__, *(u16 *)daddr);
		hdr->pci_target_id = *(u16 *)daddr;
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

	ndev->mtu = ASPEED_MCTP_MTU;
	ndev->min_mtu = MCTP_PCIE_VDM_MIN_MTU;
	ndev->max_mtu = MCTP_PCIE_VDM_MAX_MTU;
	ndev->tx_queue_len = MCTP_PCIE_VDM_TX_QUEUE_LEN;
	ndev->addr_len = 2; //PCIe bdf is 2 bytes
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

	rc = mctp_register_netdev(ndev, NULL);
	if (rc) {
		pr_err("%s: failed to register net device\n", __func__);
		free_netdev(ndev);
		return rc;
	}
	return rc;
}

static void mctp_pcie_vdm_add_dev(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct aspeed_mctp *priv = platform_get_drvdata(pdev);

	struct net_device *ndev;
	int rc;

	rc = mctp_pcie_vdm_add_net_dev(&ndev);
	if (rc) {
		pr_err("%s: failed to add net device\n", __func__);
		return;
	}

	struct mctp_pcie_vdm_dev *vdm_dev;

	vdm_dev = netdev_priv(ndev);
	vdm_dev->ndev = ndev;
	vdm_dev->dev = dev;

	rc = mctp_pcie_vdm_add_mctp_dev(vdm_dev, priv);
	if (rc) {
		pr_err("%s: failed to add mctp device\n", __func__);
		unregister_netdev(ndev);
		free_netdev(ndev);
		return;
	}
}

static void mctp_pcie_vdm_remove_dev(struct mctp_pcie_vdm_dev *vdm_dev)
{
	struct net_device *ndev = vdm_dev->ndev;

	if (ndev) {
		// TX & RX thread will be stopped in uninit operator
		mctp_unregister_netdev(ndev);
		free_netdev(ndev);
	}
}

static int mctp_pcie_vdm_scan_bounded_devices(struct device *dev, void *data)
{
	if (dev->driver) {
		if (!strcmp(dev->driver->name, "aspeed-mctp"))
			mctp_pcie_vdm_add_dev(dev);
	}
	return 0;
}

static int mctp_pcie_vdm_bus_notifier_call(struct notifier_block *nb,
					   unsigned long action, void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_BOUND_DRIVER:
		if (!strcmp(dev->driver->name, "aspeed-mctp")) {
			pr_debug("mctp platform device event %lu platform device: %s\n",
				 action, dev_name(dev));
			mctp_pcie_vdm_add_dev(dev);
		}
		break;
	case BUS_NOTIFY_UNBOUND_DRIVER:
		if (!strcmp(dev->driver->name, "aspeed-mctp")) {
			pr_debug("mctp platform device event %lu platform device: %s\n",
				 action, dev_name(dev));
			struct mctp_pcie_vdm_dev *vdm_dev;

			list_for_each_entry(vdm_dev, &mctp_pcie_vdm_devs,
					    list) {
				if (dev == vdm_dev->dev) {
					mctp_pcie_vdm_remove_dev(vdm_dev);

					mutex_lock(&mctp_pcie_vdm_dev_mutex);
					list_del(&vdm_dev->list);
					mutex_unlock(&mctp_pcie_vdm_dev_mutex);
					break;
				}
			}
		}
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static int mctp_pcie_vdm_net_notifier_call(struct notifier_block *nb,
					   unsigned long action, void *data)
{
	switch (action) {
	case MCTP_PCIE_VDM_NOTIFY_RECV:
		pr_debug("mctp pcie vdm net device event %lu data %p\n", action,
			 data);
		struct mctp_pcie_vdm_dev *vdm_dev;

		list_for_each_entry(vdm_dev, &mctp_pcie_vdm_devs, list) {
			if (vdm_dev->client == data) {
				pr_debug("mctp pcie vdm net device event %lu net device: %s\n",
					 action, vdm_dev->ndev->name);
				vdm_dev->receive_data = true;
				wake_up(&vdm_dev->rx_wait);
				break;
			}
		}
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block mctp_pcie_vdm_bus_notifier = {
	.notifier_call = mctp_pcie_vdm_bus_notifier_call,
};

static struct notifier_block mctp_pcie_vdm_net_notifier = {
	.notifier_call = mctp_pcie_vdm_net_notifier_call,
};

static __init int mctp_pcie_vdm_mod_init(void)
{
	int rc = 0;

	bus_for_each_dev(&platform_bus_type, NULL, NULL,
			 mctp_pcie_vdm_scan_bounded_devices);
	rc = bus_register_notifier(&platform_bus_type,
				   &mctp_pcie_vdm_bus_notifier);

	if (rc < 0) {
		pr_warn("mctp PCIe VDM bus notifier failed: %d\n", rc);
		return rc;
	}

	rc = mctp_pcie_vdm_register_notifier(&mctp_pcie_vdm_net_notifier);
	if (rc < 0) {
		pr_warn("mctp PCIe VDM register controller notifier failed: %d\n",
			rc);
		return rc;
	}

	return 0;
}

static __exit void mctp_pcie_vdm_mod_exit(void)
{
	int rc;

	rc = bus_unregister_notifier(&platform_bus_type,
				     &mctp_pcie_vdm_bus_notifier);
	if (rc < 0)
		pr_warn("mctp PCIe VDM could not unregister notifier, %d\n",
			rc);

	rc = mctp_pcie_vdm_unregister_notifier(&mctp_pcie_vdm_net_notifier);
	if (rc < 0)
		pr_warn("mctp PCIe VDM register controller notifier failed: %d\n",
			rc);

	struct mctp_pcie_vdm_dev *vdm_dev;

	list_for_each_entry(vdm_dev, &mctp_pcie_vdm_devs, list) {
		mctp_pcie_vdm_remove_dev(vdm_dev);

		mutex_lock(&mctp_pcie_vdm_dev_mutex);
		list_del(&vdm_dev->list);
		mutex_unlock(&mctp_pcie_vdm_dev_mutex);
	}
}

module_init(mctp_pcie_vdm_mod_init);
module_exit(mctp_pcie_vdm_mod_exit);

MODULE_DESCRIPTION("MCTP PCIe VDM transport");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("YH <yh_chung@aspeedtech.com>");
