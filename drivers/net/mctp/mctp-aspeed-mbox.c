// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <uapi/linux/if_arp.h>
#include <net/mctp.h>
#include <net/pkt_sched.h>

#define DRV_NAME "mctp_net_mailbox"

static struct net_device *mctp_dev;
static struct mbox_client mctp_mbox_client;
static struct mbox_chan *mctp_mbox_chan;
void __iomem *tx_mmio;
void __iomem *rx_mmio;
u32 tx_buf[8] = {0};

struct mctp_ipc_hdr {
	u32 msg_len;
};

static void mctp_mbox_rx_callback(struct mbox_client *cl, void *msg)
{
	struct sk_buff *skb;
	struct mctp_skb_cb *cb;
	void *buf;

	pr_debug("%s: received message from mailbox\n", DRV_NAME);

	struct mctp_ipc_hdr *hdr = (struct mctp_ipc_hdr *)msg;

	skb = netdev_alloc_skb(mctp_dev, hdr->msg_len);
	if (!skb)
		return;

	skb->protocol = htons(ETH_P_MCTP);

	pr_debug("%s: received MCTP IPC SHM packet %p\n", DRV_NAME, rx_mmio);

	buf = kzalloc(hdr->msg_len, GFP_ATOMIC);
	if (!buf)
		return;
	memcpy_fromio(buf, rx_mmio, hdr->msg_len);

	skb_put_data(skb, buf, hdr->msg_len);

	kfree(buf);

	skb_reset_network_header(skb);

	cb = __mctp_cb(skb);
	cb->halen = 0;

	netif_rx(skb);
	mctp_dev->stats.rx_packets++;
	mctp_dev->stats.rx_bytes += hdr->msg_len;
}

static void mctp_mbox_tx_prepare(struct mbox_client *cl, void *msg)
{
	pr_debug("%s: preparing message for transmission\n", DRV_NAME);
}

static void mctp_mbox_tx_done(struct mbox_client *cl, void *msg, int r)
{
	struct mctp_ipc_hdr *hdr = (struct mctp_ipc_hdr *)msg;

	pr_debug("%s: transmission completed with status %d\n", DRV_NAME, r);
	mctp_dev->stats.tx_packets++;
	mctp_dev->stats.tx_bytes += hdr->msg_len;
	netif_wake_queue(mctp_dev);
}

static int mctp_mbox_net_open(struct net_device *dev)
{
	pr_debug("%s: device opened\n", dev->name);
	return 0;
}

static int mctp_mbox_net_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	pr_debug("%s: device stopped\n", dev->name);
	return 0;
}

static netdev_tx_t mctp_mbox_net_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned int plen;
	struct mctp_ipc_hdr *hdr;
	int rc;

	if (!mctp_mbox_chan) {
		dev_kfree_skb(skb);
		return NETDEV_TX_BUSY;
	}

	plen = skb->len;
	rc = skb_cow_head(skb, 4);
	if (rc) {
		dev_kfree_skb(skb);
		return NETDEV_TX_BUSY;
	}

	hdr = skb_push(skb, 4);
	hdr->msg_len = plen;

	memcpy_toio(tx_mmio, skb->data + 4, plen);
	memcpy(tx_buf, hdr, sizeof(*hdr));

	mbox_send_message(mctp_mbox_chan, tx_buf);

	dev_kfree_skb(skb);
	netif_stop_queue(dev);
	return NETDEV_TX_OK;
}

static const struct net_device_ops mctp_mbox_netdev_ops = {
	.ndo_open = mctp_mbox_net_open,
	.ndo_stop = mctp_mbox_net_stop,
	.ndo_start_xmit = mctp_mbox_net_start_xmit,
};

static void mctp_mbox_net_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;
	dev->mtu = 1024;
	dev->min_mtu = 16;
	dev->max_mtu = 1024 * 1024 * 1;
	dev->hard_header_len = 4;
	dev->tx_queue_len = DEFAULT_TX_QUEUE_LEN;
	dev->flags = IFF_NOARP;
	dev->netdev_ops = &mctp_mbox_netdev_ops;
	dev->pcpu_stat_type = NETDEV_PCPU_STAT_DSTATS;
}

static int mctp_mbox_net_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;

	mctp_mbox_client.dev = &pdev->dev;
	mctp_mbox_client.rx_callback = mctp_mbox_rx_callback;
	mctp_mbox_client.tx_prepare = mctp_mbox_tx_prepare;
	mctp_mbox_client.tx_done = mctp_mbox_tx_done;
	mctp_mbox_client.tx_block = false;
	mctp_mbox_client.tx_tout = 0;
	mctp_mbox_client.knows_txdone = false;

	tx_mmio = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(tx_mmio)) {
		dev_err(&pdev->dev, "Failed to get and map tx MMIO resource\n");
		return PTR_ERR(tx_mmio);
	}

	rx_mmio = devm_platform_get_and_ioremap_resource(pdev, 1, &res);
	if (PTR_ERR(rx_mmio) == -EBUSY)
		rx_mmio = devm_ioremap(&pdev->dev, res->start, 0x1000);

	dev_info(&pdev->dev, "tx_mmio=%p, rx_mmio=%p\n", tx_mmio, rx_mmio);

	mctp_mbox_chan = mbox_request_channel(&mctp_mbox_client, 0);
	if (IS_ERR(mctp_mbox_chan)) {
		pr_err("%s: failed to request mailbox channel\n", DRV_NAME);
		return PTR_ERR(mctp_mbox_chan);
	}

	mctp_dev = alloc_netdev(0, "mctpmbox%d", NET_NAME_ENUM, mctp_mbox_net_setup);
	if (!mctp_dev) {
		pr_err("%s: failed to allocate netdev\n", DRV_NAME);
		return -ENOMEM;
	}

	mctp_dev->dev.parent = &pdev->dev;
	mctp_dev->dev.of_node = pdev->dev.of_node;

	ret = register_netdev(mctp_dev);
	if (ret) {
		pr_err("%s: failed to register netdev\n", DRV_NAME);
		free_netdev(mctp_dev);
		return ret;
	}

	platform_set_drvdata(pdev, mctp_dev);
	pr_info("%s: device registered as %s\n", DRV_NAME, mctp_dev->name);
	return 0;
}

static void mctp_mbox_net_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	unregister_netdev(dev);
	free_netdev(dev);
}

static const struct of_device_id mctp_net_of_match[] = {
	{ .compatible = "aspeed,mctp-net-mailbox" },
	{ },
};

MODULE_DEVICE_TABLE(of, mctp_net_of_match);

static struct platform_driver mctp_mbox_net_driver = {
	.probe = mctp_mbox_net_probe,
	.remove = mctp_mbox_net_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = mctp_net_of_match,
	},
};

module_platform_driver(mctp_mbox_net_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Troy Lee");
MODULE_DESCRIPTION("MCTP Aspeed Mailbox transport");
