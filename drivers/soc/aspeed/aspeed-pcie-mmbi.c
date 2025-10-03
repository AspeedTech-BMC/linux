// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) ASPEED Technology Inc.

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>

#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/mctp.h>
#include <net/mctp.h>
#include <net/pkt_sched.h>

#include "aspeed-pcie-mmbi.h"

/* AST2700 E2M */
#define ASPEED_E2M_EVENT 0x0D0
#define ASPEED_E2M_EVENT_SET 0x0D4
#define ASPEED_E2M_EVENT_CLR 0x0D8
#define ASPEED_E2M_EVENT_EN 0x0DC
#define ASPEED_E2M_ADRMAP00 0x100
#define ASPEED_E2M_WIRQA0 0x180
#define ASPEED_E2M_WIRQV0 0x1C0
#define ASPEED_E2M_SPROT_SIDG0 0x210
#define ASPEED_E2M_SPROT_CTL0 0x280
#define ASPEED_E2M_SPROT_ADR0 0x2C0

/* AST2700 SCU */
#define ASPEED_SCU_DECODE_DEV BIT(18)
#define ASPEED_SCU_INT_EN BIT(23)
struct aspeed_platform {
	int (*mmbi_init)(struct platform_device *pdev);
};

struct aspeed_pcie_mmbi {
	struct device *dev;
	struct regmap *device;
	struct regmap *e2m;
	int irq;
	const struct aspeed_platform *platform;
	/* E2M index */
	int id;
	int pid;
	int scu_bar_offset;
	int e2m_index;
	int e2m_h2b_int;

	/* Memory Mapping */
	void __iomem *mem_virt;
	dma_addr_t mem_phy;
	phys_addr_t mem_size;

	struct aspeed_mmbi_channel chan;
};

static void mmbi_desc_init(struct aspeed_mmbi_channel *chan);

static u8 mmbi_get_bmc_up(struct aspeed_mmbi_channel *chan)
{
	struct host_ros hros;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));

	return hros.b_up;
}

static u8 mmbi_get_bmc_rdy(struct aspeed_mmbi_channel *chan)
{
	struct host_ros hros;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));

	return hros.b_rdy;
}

static u8 mmbi_get_bmc_rst(struct aspeed_mmbi_channel *chan)
{
	struct host_ros hros;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));

	return hros.b_rst;
}

static u8 mmbi_get_host_rst(struct aspeed_mmbi_channel *chan)
{
	struct host_rws hrws;

	memcpy_fromio(&hrws, chan->hrws_vmem, sizeof(hrws));

	return hrws.h_rst;
}

static u8 mmbi_get_host_rdy(struct aspeed_mmbi_channel *chan)
{
	struct host_rws hrws;

	memcpy_fromio(&hrws, chan->hrws_vmem, sizeof(hrws));

	return hrws.h_rdy;
}

static u8 mmbi_get_host_up(struct aspeed_mmbi_channel *chan)
{
	struct host_rws hrws;

	memcpy_fromio(&hrws, chan->hrws_vmem, sizeof(hrws));

	return hrws.h_up;
}

static void mmbi_set_bmc_rst(struct aspeed_mmbi_channel *chan, bool set)
{
	struct host_ros hros;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));
	hros.b_rst = set;
	memcpy_toio(chan->hros_vmem, &hros, sizeof(hros));
}

static void mmbi_set_bmc_rdy(struct aspeed_mmbi_channel *chan, bool set)
{
	struct host_ros hros;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));
	hros.b_rdy = set;
	memcpy_toio(chan->hros_vmem, &hros, sizeof(hros));
}

static void mmbi_set_bmc_up(struct aspeed_mmbi_channel *chan, bool set)
{
	struct host_ros hros;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));
	hros.b_up = set;
	memcpy_toio(chan->hros_vmem, &hros, sizeof(hros));
}

static void raise_b2h_interrupt(struct aspeed_mmbi_channel *chan)
{
	if (!chan->host_int_en)
		return;

	regmap_write(chan->mmbi->e2m, ASPEED_E2M_EVENT_SET, BIT(chan->mmbi->e2m_h2b_int));
}

static void mmbi_clear_hros(struct aspeed_mmbi_channel *chan)
{
	memset_io(chan->hros_vmem, 0, sizeof(struct host_ros));
}

static void mmbi_clear_hrws(struct aspeed_mmbi_channel *chan)
{
	memset_io(chan->hrws_vmem, 0, sizeof(struct host_rws));
}

static void get_b2h_avail_buf_len(struct aspeed_mmbi_channel *chan, ssize_t *avail_buf_len)
{
	struct device *dev = chan->dev;
	u32 b2h_rp, b2h_wp;

	b2h_rp = GET_B2H_READ_POINTER(chan);
	b2h_wp = GET_B2H_WRITE_POINTER(chan);
	dev_dbg(dev, "MMBI B2H - b2h_rp: 0x%0x, b2h_wp: 0x%0x\n", b2h_rp, b2h_wp);

	if (b2h_wp >= b2h_rp)
		*avail_buf_len = chan->b2h_cb_size - b2h_wp + b2h_rp;
	else
		*avail_buf_len = b2h_rp - b2h_wp;
}

static u8 mmbi_get_state(struct aspeed_mmbi_channel *chan)
{
	u8 state = 0;

	state = mmbi_get_bmc_up(chan) << 3;
	state |= mmbi_get_bmc_rst(chan) << 2;
	state |= mmbi_get_host_up(chan) << 1;
	state |= mmbi_get_host_rst(chan);

	return state;
}

static int get_mmbi_header(struct aspeed_mmbi_channel *chan, u32 *data_length,
			   u8 *type, u32 *unread_data_len, u8 *padding)
{
	u32 h2b_wp, h2b_rp, b2h_wp, b2h_rp;
	struct device *dev = chan->dev;
	struct mmbi_header header;

	h2b_wp = GET_H2B_WRITE_POINTER(chan);
	h2b_rp = GET_H2B_READ_POINTER(chan);
	b2h_wp = GET_B2H_WRITE_POINTER(chan);
	b2h_rp = GET_B2H_READ_POINTER(chan);
	dev_dbg(dev, "MMBI HRWS - h2b_wp: 0x%0x, b2h_rp: 0x%0x\n", h2b_wp,
		b2h_rp);
	dev_dbg(dev, "MMBI HROS - b2h_wp: 0x%0x, h2b_rp: 0x%0x\n", b2h_wp,
		h2b_rp);

	if (h2b_wp >= h2b_rp)
		*unread_data_len = h2b_wp - h2b_rp;
	else
		*unread_data_len = chan->h2b_cb_size - h2b_rp + h2b_wp;

	if (*unread_data_len < sizeof(struct mmbi_header)) {
		dev_dbg(dev, "No data to read(%d - %d)\n", h2b_wp, h2b_rp);
		return -EAGAIN;
	}

	dev_dbg(dev, "READ MMBI header from: 0x%lx\n",
		(ssize_t)(chan->h2b_cb_vmem + h2b_rp));

	/* Extract MMBI protocol - protocol type and length */
	if ((h2b_rp + sizeof(header)) <= chan->h2b_cb_size) {
		memcpy_fromio(&header, chan->h2b_cb_vmem + h2b_rp,
			      sizeof(header));
	} else {
		ssize_t chunk_len = chan->h2b_cb_size - h2b_rp;

		memcpy_fromio(&header, chan->h2b_cb_vmem + h2b_rp, chunk_len);
		memcpy_fromio(((u8 *)&header) + chunk_len, chan->h2b_cb_vmem,
			      sizeof(header) - chunk_len);
	}

	*data_length = (header.pkt_len << 2) - sizeof(header) - header.pkt_pad;
	*padding = header.pkt_pad;
	*type = header.pkt_type;

	return 0;
}

static int mmbi_state_check(struct aspeed_mmbi_channel *chan)
{
	enum mmbi_state current_state = mmbi_get_state(chan);
	struct device *dev = chan->dev;
	u32 req_data_len, unread_data_len;
	u8 type, padding;

	switch (current_state) {
	case INIT_MISMATCH:
		dev_dbg(dev, "Get INIT_MISMATCH state from HOST");
		/* Reset MMBI data structure */
		mmbi_desc_init(chan);
		/* Translat state to INIT_IN_PROGRESS */
		mmbi_clear_hros(chan);
		mmbi_clear_hrws(chan);
		/* Translat state to INIT_COMPLETED*/
		mmbi_set_bmc_up(chan, 1);

		dev_dbg(dev, "Change state to INIT_COMPLETED to HOST");
		raise_b2h_interrupt(chan);
		return 1;
	case NORMAL_RUNTIME:
		if (mmbi_get_bmc_rdy(chan))
			return 0;
		dev_dbg(dev, "Get NORMAL_RUNTIME state from HOST");
		mmbi_set_bmc_rdy(chan, 1);
		return 1;
	case RESET_REQ_BY_HOST:
		dev_dbg(dev, "Get RESET_REQ_BY_HOST state from HOST");
		/* Stop operation */
		mmbi_set_bmc_rdy(chan, 0);
		/* Change state to RESET_ACKED */
		mmbi_set_bmc_rst(chan, 1);
		raise_b2h_interrupt(chan);
		/* Change state to TRANS_TO_INIT */
		mmbi_set_bmc_up(chan, 0);
		/* Reset MMBI data structure */
		mmbi_desc_init(chan);
		/* Translat state to INIT_IN_PROGRESS */
		mmbi_clear_hros(chan);
		mmbi_clear_hrws(chan);
		/* Translat state to INIT_COMPLETED*/
		mmbi_set_bmc_up(chan, 1);

		dev_dbg(dev, "Change state to INIT_COMPLETED to HOST");
		raise_b2h_interrupt(chan);
		return 1;
	case RESET_ACKED:
		/* Receive all packet from Host */
		while (get_mmbi_header(chan, &req_data_len, &type, &unread_data_len, &padding) == 0 &&
		       mmbi_get_state(chan) == RESET_ACKED)
			;
		/* Change state to TRANS_TO_INIT */
		mmbi_set_bmc_up(chan, 0);
		/* Reset MMBI data structure */
		mmbi_desc_init(chan);
		/* Translat state to INIT_IN_PROGRESS */
		mmbi_clear_hros(chan);
		mmbi_clear_hrws(chan);
		/* Translat state to INIT_COMPLETED*/
		mmbi_set_bmc_up(chan, 1);

		dev_dbg(dev, "Change state to INIT_COMPLETED to HOST");
		raise_b2h_interrupt(chan);
	default:
		break;
	}

	return 0;
}

static void update_host_ros(struct aspeed_mmbi_channel *chan, unsigned int w_len,
			    unsigned int r_len)
{
	struct device *dev = chan->dev;
	struct host_ros hros;
	u32 h2b_rp, b2h_wp;

	b2h_wp = GET_B2H_WRITE_POINTER(chan);
	h2b_rp = GET_H2B_READ_POINTER(chan);

	/* Advance the B2H CB offset for next write */
	if ((b2h_wp + w_len) <= chan->b2h_cb_size)
		b2h_wp += w_len;
	else
		b2h_wp = b2h_wp + w_len - chan->b2h_cb_size;

	/* Advance the H2B CB offset till where BMC read data */
	if ((h2b_rp + r_len) <= chan->h2b_cb_size)
		h2b_rp += r_len;
	else
		h2b_rp = h2b_rp + r_len - chan->h2b_cb_size;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));
	hros.b2h_wp = FIELD_GET(B2H_WRITE_POINTER_MASK, b2h_wp);
	hros.h2b_rp = FIELD_GET(H2B_READ_POINTER_MASK, h2b_rp);
	memcpy_toio(chan->hros_vmem, &hros, sizeof(hros));
	dev_dbg(dev, "Updating HROS - h2b_rp: 0x%0x, b2h_wp: 0x%0x\n", h2b_rp, b2h_wp);

	if (w_len != 0)
		raise_b2h_interrupt(chan);
}

static int aspeed_mmbi_write(struct aspeed_mmbi_channel *chan, char *buffer, size_t len,
			     protocol_type type)
{
	struct device *dev = chan->dev;
	struct mmbi_header header = { 0 };
	ssize_t avail_buf_len;
	ssize_t total_len;
	ssize_t wt_offset;
	ssize_t chunk_len;
	ssize_t end_offset;
	u8 padding = 0;

	/* If HOST READY bit is not set, Just discard the write. */
	if (!GET_HOST_READY_BIT(chan)) {
		dev_dbg(dev, "Host not ready, discarding request...\n");
		return -EAGAIN;
	}

	get_b2h_avail_buf_len(chan, &avail_buf_len);

	dev_dbg(dev, "B2H buffer empty space: %ld\n", avail_buf_len);

	/* Header size */
	total_len = len + 4;

	padding = total_len & 0x3;
	if (padding)
		padding = 4 - padding;
	total_len += padding;

	/* Empty space should be more than write request data size */
	if (avail_buf_len <= sizeof(header) || (total_len > (avail_buf_len - sizeof(header))))
		return -ENOSPC;

	/* Fill multi-protocol header */
	header.pkt_type = type;
	header.pkt_len = total_len >> 2;
	header.pkt_pad = padding;

	wt_offset = GET_B2H_WRITE_POINTER(chan);
	end_offset = chan->b2h_cb_size;

	/* Copy Header */
	if ((end_offset - wt_offset) >= sizeof(header)) {
		memcpy_toio(chan->b2h_cb_vmem + wt_offset, &header, sizeof(header));
		wt_offset += sizeof(header);
	} else {
		chunk_len = end_offset - wt_offset;
		memcpy_toio(chan->b2h_cb_vmem + wt_offset, &header, chunk_len);
		memcpy_toio(chan->b2h_cb_vmem, &header + chunk_len, (sizeof(header) - chunk_len));
		wt_offset = (sizeof(header) - chunk_len);
	}

	/* Write the data */
	if ((end_offset - wt_offset) >= len) {
		memcpy_toio(&chan->b2h_cb_vmem[wt_offset], buffer, len);
		wt_offset += len;
	} else {
		chunk_len = end_offset - wt_offset;
		dev_dbg(dev, "Write data chunk_len: %ld\n", chunk_len);
		memcpy_toio(&chan->b2h_cb_vmem[wt_offset], buffer, chunk_len);

		wt_offset = 0;
		memcpy_toio(&chan->b2h_cb_vmem[wt_offset], buffer + chunk_len, len - chunk_len);
		wt_offset += len - chunk_len;
	}

	update_host_ros(chan, total_len, 0);

	return 0;
}

static void aspeed_mmbi_read(struct aspeed_mmbi_channel *chan, char *buffer, size_t len, u8 padding)
{
	struct device *dev = chan->dev;
	ssize_t rd_offset;
	u32 h2b_rp;

	h2b_rp = GET_H2B_READ_POINTER(chan);
	if ((h2b_rp + sizeof(struct mmbi_header)) <= chan->h2b_cb_size)
		rd_offset = h2b_rp + sizeof(struct mmbi_header);
	else
		rd_offset = h2b_rp + sizeof(struct mmbi_header) - chan->h2b_cb_size;

	/* Extract data and copy to user space application */
	dev_dbg(dev, "READ MMBI Data from: 0x%0lx and length: %ld\n",
		(ssize_t)(chan->h2b_cb_vmem + rd_offset), len);

	if ((chan->h2b_cb_size - rd_offset) >= len) {
		memcpy_fromio(buffer, chan->h2b_cb_vmem + rd_offset, len);
		rd_offset += len;
	} else {
		ssize_t chunk_len;

		chunk_len = chan->h2b_cb_size - rd_offset;
		dev_dbg(dev, "Read data chunk_len: %ld\n", chunk_len);
		memcpy_fromio(buffer, chan->h2b_cb_vmem + rd_offset, chunk_len);

		rd_offset = 0;
		memcpy_fromio(buffer + chunk_len, chan->h2b_cb_vmem + rd_offset, len - chunk_len);
	}

	update_host_ros(chan, 0, len + sizeof(struct mmbi_header) + padding);
}

static void mctp_mmbi_rx(struct aspeed_mmbi_channel *chan)
{
	struct net_device *ndev = chan->ndev;
	struct sk_buff *skb;
	struct mctp_skb_cb *cb;
	u32 req_data_len, unread_data_len;
	u8 type, padding;
	int status;

	if (get_mmbi_header(chan, &req_data_len, &type, &unread_data_len, &padding) != 0)
		return;

	dev_dbg(chan->dev, "%s: Length: 0x%0x, Protocol Type: %d, Unread data: %d\n", __func__,
		req_data_len, type, unread_data_len);

	skb = netdev_alloc_skb(ndev, req_data_len);
	if (!skb) {
		ndev->stats.rx_dropped++;
		update_host_ros(chan, 0, req_data_len + sizeof(struct mmbi_header));
		return;
	}

	skb->protocol = htons(ETH_P_MCTP);
	aspeed_mmbi_read(chan, skb_put(skb, req_data_len), req_data_len, padding);
	skb_reset_network_header(skb);

	cb = __mctp_cb(skb);
	cb->halen = 0;

	status = netif_rx(skb);
	if (status == NET_RX_SUCCESS) {
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += req_data_len;
	} else {
		ndev->stats.rx_dropped++;
	}
}

static netdev_tx_t mctp_mmbi_tx(struct sk_buff *skb, struct net_device *ndev)
{
	struct aspeed_mmbi_mctp *mctp = netdev_priv(ndev);
	int ret;

	if (!mmbi_get_host_rdy(&mctp->mmbi->chan) || skb->len > MCTP_MMBI_MTU_MAX) {
		ndev->stats.tx_dropped++;
		goto out;
	}

	ret = aspeed_mmbi_write(&mctp->mmbi->chan, skb->data, skb->len, MMBI_PROTOCOL_MCTP);
	if (ret) {
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops mctp_mmbi_netdev_ops = {
	.ndo_start_xmit = mctp_mmbi_tx,
};

static void aspeed_mctp_mmbi_setup(struct net_device *ndev)
{
	ndev->type = ARPHRD_MCTP;

	/* we limit at the fixed MTU, which is also the MCTP-standard
	 * baseline MTU, so is also our minimum
	 */
	ndev->mtu = MCTP_MMBI_MTU;
	ndev->max_mtu = MCTP_MMBI_MTU_MAX;
	ndev->min_mtu = MCTP_MMBI_MTU_MIN;

	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->tx_queue_len = DEFAULT_TX_QUEUE_LEN;
	ndev->flags = IFF_NOARP;
	ndev->netdev_ops = &mctp_mmbi_netdev_ops;
	ndev->needs_free_netdev = true;
}

static int aspeed_mmbi_mctp_init(struct aspeed_mmbi_channel *chan)
{
	struct aspeed_mmbi_mctp *mctp;
	struct net_device *ndev;
	char name[32];
	int ret;

	snprintf(name, sizeof(name), "mctpmmbi%d%d", chan->mmbi->id, chan->mmbi->e2m_index);
	ndev = alloc_netdev(sizeof(*mctp), name, NET_NAME_ENUM, aspeed_mctp_mmbi_setup);
	if (!ndev)
		return -ENOMEM;

	mctp = netdev_priv(ndev);
	mctp->ndev = ndev;
	mctp->mmbi = chan->mmbi;

	chan->ndev = ndev;

	ret = register_netdev(ndev);
	if (ret)
		goto free_netdev;

	return 0;

free_netdev:
	free_netdev(ndev);

	return ret;
}

static void aspeed_mmbi_work_func(struct work_struct *workq)
{
	struct aspeed_mmbi_channel *chan = container_of(workq, struct aspeed_mmbi_channel, work);
	u32 weight = 256, req_data_len, unread_data_len;
	u8 type, padding;
	int i;

	for (i = 0; i < weight; i++) {
		if (get_mmbi_header(chan, &req_data_len, &type, &unread_data_len, &padding) != 0)
			return;

		dev_dbg(chan->dev, "%s: Length: 0x%0x, Protocol Type: %d\n",
			__func__, req_data_len, type);

		if (type == MMBI_PROTOCOL_MCTP)
			mctp_mmbi_rx(chan);
		else
			/* Discard data and advance the hrws */
			update_host_ros(chan, 0, req_data_len + sizeof(struct mmbi_header) + padding);

		raise_b2h_interrupt(chan);
	}

	if (get_mmbi_header(chan, &req_data_len, &type, &unread_data_len, &padding) != 0)
		queue_work(system_unbound_wq, &chan->work);
}

static irqreturn_t aspeed_pcie_mmbi_isr(int irq, void *dev_id)
{
	struct aspeed_pcie_mmbi *mmbi = dev_id;
	struct aspeed_mmbi_channel *chan = &mmbi->chan;
	ssize_t avail_buf_len;

	get_b2h_avail_buf_len(chan, &avail_buf_len);
	if (avail_buf_len > MCTP_MMBI_MTU_MAX) {
		if (netif_queue_stopped(chan->ndev)) {
			dev_dbg(chan->dev, "Wake up mctp net device\n");
			netif_wake_queue(chan->ndev);
		}
	}

	if (mmbi_state_check(chan))
		return IRQ_HANDLED;

	queue_work(system_unbound_wq, &chan->work);

	return IRQ_HANDLED;
}

static void mmbi_desc_init(struct aspeed_mmbi_channel *chan)
{
	struct mmbi_cap_desc desc;

	memset(&desc, 0, sizeof(struct mmbi_cap_desc));

	desc.version = 1;
	/* This MMBI interface is intended for OS use */
	desc.os_use = 1;
	desc.b2h_ba = (chan->b2h_cb_vmem - chan->desc_vmem) >> 3;
	desc.h2b_ba = (chan->h2b_cb_vmem - chan->desc_vmem) >> 3;
	/* Make sure the buffer size is 4 byte aligmnent */
	desc.b2h_l = chan->b2h_cb_size & ~0x3;
	desc.h2b_l = chan->h2b_cb_size & ~0x3;
	/* Variable Packet Size Circular Buffers (VPSCB) v1 */
	desc.buffer_type = 0x01;
	desc.bt_desc.h_ros_p = (chan->hros_vmem - chan->desc_vmem) >> 3;
	desc.bt_desc.h_rws_p = (chan->hrws_vmem - chan->desc_vmem) >> 3;
	/* PCIe Interrupt */
	desc.bt_desc.h_int_t = 0x01;
	desc.bt_desc.h_int_l = chan->host_int_location;
	desc.bt_desc.h_int_v = 0; /* Skip for PCIe Interrupt */
	desc.bt_desc.bmc_int_t = 0x01; /* relative memory space address */
	desc.bt_desc.bmc_int_l = chan->bmc_int_location;
	desc.bt_desc.bmc_int_v = chan->bmc_int_value;

	/* Per MMBI protoco spec, Set it to "#MMBI$" */
	memcpy(desc.signature, MMBI_SIGNATURE, sizeof(desc.signature));

	memcpy_toio(chan->desc_vmem, &desc, sizeof(desc));
}

static int aspeed_pcie_mmbi_init(struct aspeed_pcie_mmbi *mmbi)
{
	struct aspeed_mmbi_channel *chan = &mmbi->chan;
	struct device *dev = chan->dev;
	u32 b2h_size = mmbi->mem_size >> 1;
	u32 h2b_size = mmbi->mem_size >> 1;
	u8 *h2b_vaddr, *b2h_vaddr;
	int ret;

	b2h_vaddr = mmbi->mem_virt;
	h2b_vaddr = b2h_vaddr + b2h_size;

	chan->dev = dev;
	chan->desc_vmem = b2h_vaddr;
	chan->hros_vmem = b2h_vaddr + sizeof(struct mmbi_cap_desc);
	chan->b2h_cb_vmem = b2h_vaddr + sizeof(struct mmbi_cap_desc) + sizeof(struct host_ros);
	chan->b2h_cb_size = b2h_size - sizeof(struct mmbi_cap_desc) - sizeof(struct host_ros);

	chan->hrws_vmem = h2b_vaddr;
	chan->h2b_cb_vmem = h2b_vaddr + sizeof(struct host_rws);
	chan->h2b_cb_size = h2b_size - sizeof(struct host_rws);

	dev_dbg(dev, "B2H mapped addr - desc: 0x%0lx, hros: 0x%0lx, b2h_cb: 0x%0lx\n",
		(size_t)chan->desc_vmem, (size_t)chan->hros_vmem, (size_t)chan->b2h_cb_vmem);
	dev_dbg(dev, "H2B mapped addr - hrws: 0x%0lx, h2b_cb: 0x%0lx\n", (size_t)chan->hrws_vmem,
		(size_t)chan->h2b_cb_vmem);

	dev_dbg(dev, "B2H buffer size: 0x%0lx\n", (size_t)chan->b2h_cb_size);
	dev_dbg(dev, "H2B buffer size: 0x%0lx\n", (size_t)chan->h2b_cb_size);

	/* Initialize the MMBI channel descriptor */
	mmbi_desc_init(chan);

	/* Clear HRWS & HROS */
	mmbi_clear_hros(chan);
	mmbi_clear_hrws(chan);

	/* Initialize MTCP function */
	ret = aspeed_mmbi_mctp_init(chan);
	if (ret) {
		dev_err(dev, "Unable to init mctp\n");
		return ret;
	}

	/* Set BMC UP bit */
	mmbi_set_bmc_up(chan, 1);

	return 0;
}

/*
 * AST2700 PCIe MMBI (SCU & E2M)
 * SoC         |    0                                    |    1                          |
 * PCI class   |    MFD (0xFF_00_00)                     |    MMBI (0x0C_0C_00)          |
 * Node        |    0                   1                |    0                          |
 * PID         |    3    4    5    6   11   12   13   14 |    2    3    4    5    6    7 |
 * E2M index   |    0    1    2    3    4    5    6    7 |    0    1    2    3    4    5 |
 * BAR index   |    2    3    4    5    2    3    4    5 |    0    1    2    3    4    5 |
 * SCU BAR     |   3c   4c   5c   6c   3c   4c   5c   6c |   1c   50   3c   4c   5c   6c |
 * E2M H2B Int |    0    1    2    3    0    1    2    3 |    0    1    2    3    4    5 | (bit)
 */
static int aspeed_ast2700_pcie_mmbi_init(struct platform_device *pdev)
{
	struct aspeed_pcie_mmbi *mmbi = platform_get_drvdata(pdev);
	struct aspeed_mmbi_channel *chan = &mmbi->chan;
	struct device *dev = &pdev->dev;
	u32 value, sprot_size, e2m_index, pid;
	struct resource res;
	int ret, i;

	/* Get register map*/
	mmbi->e2m = syscon_node_to_regmap(dev->of_node->parent);
	if (IS_ERR(mmbi->e2m)) {
		dev_err(dev, "failed to find e2m regmap\n");
		return PTR_ERR(mmbi->e2m);
	}
	if (of_address_to_resource(dev->of_node->parent, 0, &res)) {
		dev_err(dev, "Failed to get e2m resource\n");
		return -EINVAL;
	}
	if (res.start == 0x14c1d000)
		mmbi->id = 2;
	else if (res.start == 0x12c22000)
		mmbi->id = 1;
	else
		mmbi->id = 0; /* 0x12c21000 */

	mmbi->device = syscon_regmap_lookup_by_phandle(dev->of_node->parent, "aspeed,device");
	if (IS_ERR(mmbi->device)) {
		dev_err(dev, "failed to find device regmap\n");
		return PTR_ERR(mmbi->device);
	}

	ret = of_property_read_u32(dev->of_node, "index", &mmbi->e2m_index);
	if (ret < 0) {
		dev_err(dev, "cannot get mmbi index value\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "pid", &mmbi->pid);
	if (ret < 0) {
		dev_err(dev, "cannot get mmbi pid value\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "bar", &mmbi->scu_bar_offset);
	if (ret < 0) {
		dev_err(dev, "cannot get mmbi bar value\n");
		return ret;
	}

	e2m_index = mmbi->e2m_index;
	pid = mmbi->pid;
	mmbi->e2m_h2b_int += mmbi->e2m_index;
	if (mmbi->id < 2) {
		/* PCIe device class, sub-class, protocol and reversion */
		regmap_write(mmbi->device, 0x18, 0xFF000027);
	} else {
		regmap_write(mmbi->device, 0x18, 0x0C0C0027);
		regmap_write(mmbi->device, 0x78, ASPEED_SCU_INT_EN | ASPEED_SCU_DECODE_DEV);
	}

	/* MSI */
	regmap_update_bits(mmbi->device, 0x74, GENMASK(7, 4), BIT(7) | (5 << 4));

	regmap_update_bits(mmbi->device, 0x70, BIT(25) | BIT(17) | BIT(9) | BIT(1),
			   BIT(25) | BIT(17) | BIT(9) | BIT(1));

	/* Calculate the BAR Size */
	for (i = 1; i < 16; i++) {
		/* bar size check for 4k align */
		if ((mmbi->mem_size / 4096) == (1 << (i - 1)))
			break;
	}
	if (i == 16) {
		i = 0;
		dev_warn(dev, "Bar size not align for 4K : %dK\n", (u32)mmbi->mem_size / 1024);
	}
	regmap_write(mmbi->device, mmbi->scu_bar_offset, (mmbi->mem_phy >> 4) | i);
	regmap_write(mmbi->e2m, ASPEED_E2M_ADRMAP00 + (4 * pid), (mmbi->mem_phy >> 4) | i);

	/* BMC Interrupt */
	if (chan->bmc_int_en) {
		value = mmbi->mem_phy + chan->bmc_int_location;
		regmap_write(mmbi->e2m, ASPEED_E2M_WIRQA0 + (4 * e2m_index), value);
		value = (BIT(16) << pid) | chan->bmc_int_value;
		regmap_write(mmbi->e2m, ASPEED_E2M_WIRQV0 + (4 * e2m_index), value);
	}

	/* HOST Interrupt: MSI */
	regmap_read(mmbi->e2m, ASPEED_E2M_EVENT_EN, &value);
	value |= BIT(mmbi->e2m_h2b_int);
	regmap_write(mmbi->e2m, ASPEED_E2M_EVENT_EN, value);

	/* B2H Write Protect */
	sprot_size = (mmbi->mem_size / 2) / SZ_1M;
	value = (sprot_size << 16) | (mmbi->mem_phy >> 20);
	regmap_write(mmbi->e2m, ASPEED_E2M_SPROT_ADR0 + (4 * e2m_index), value);
	/* Enable read & disalbe write */
	value = 1 << (8 + e2m_index);
	regmap_write(mmbi->e2m, ASPEED_E2M_SPROT_CTL0 + (4 * e2m_index), value);
	/* Set PID */
	regmap_read(mmbi->e2m, ASPEED_E2M_SPROT_SIDG0 + (4 * (e2m_index / 4)), &value);
	value |= pid << (8 * (e2m_index % 4));
	regmap_write(mmbi->e2m, ASPEED_E2M_SPROT_SIDG0 + (4 * (e2m_index / 4)), value);

	mmbi->chan.dev = dev;
	mmbi->chan.mmbi = mmbi;
	ret = aspeed_pcie_mmbi_init(mmbi);
	if (ret < 0) {
		dev_err(dev, "Initialize MMBI device failed.\n");
		return ret;
	}

	INIT_WORK(&chan->work, aspeed_mmbi_work_func);

	return 0;
}

struct aspeed_platform ast2700_platform = {
	.mmbi_init = aspeed_ast2700_pcie_mmbi_init,
};

static const struct of_device_id aspeed_pcie_mmbi_of_matches[] = {
	{ .compatible = "aspeed,ast2700-pcie-mmbi", .data = &ast2700_platform },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_pcie_mmbi_of_matches);

static int aspeed_pcie_mmbi_probe(struct platform_device *pdev)
{
	struct aspeed_pcie_mmbi *mmbi;
	struct aspeed_mmbi_channel *chan;
	struct device *dev = &pdev->dev;
	struct resource res;
	struct device_node *np;
	const void *md;
	int ret = 0;

	md = of_device_get_match_data(dev);
	if (!md)
		return -ENODEV;

	mmbi = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_pcie_mmbi), GFP_KERNEL);
	if (!mmbi)
		return -ENOMEM;
	dev_set_drvdata(dev, mmbi);

	mmbi->dev = dev;
	mmbi->platform = md;

	/* Get MMBI memory size */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np || of_address_to_resource(np, 0, &res)) {
		dev_err(dev, "Failed to find memory-region.\n");
		ret = -ENOMEM;
		goto out_region;
	}

	of_node_put(np);

	mmbi->mem_phy = res.start;
	mmbi->mem_size = resource_size(&res);
	mmbi->mem_virt = ioremap(mmbi->mem_phy, mmbi->mem_size);
	if (!mmbi->mem_virt) {
		dev_err(dev, "cannot map mmbi memory region\n");
		ret = -ENOMEM;
		goto out_region;
	}

	/* Get IRQ */
	mmbi->irq = platform_get_irq(pdev, 0);
	if (mmbi->irq < 0) {
		dev_err(&pdev->dev, "platform get of irq[=%d] failed!\n", mmbi->irq);
		goto out_unmap;
	}
	ret = devm_request_irq(&pdev->dev, mmbi->irq, aspeed_pcie_mmbi_isr, 0, dev_name(&pdev->dev),
			       mmbi);
	if (ret) {
		dev_err(dev, "pcie mmbi unable to get IRQ");
		goto out_unmap;
	}

	chan = &mmbi->chan;
	memset(chan, 0, sizeof(struct aspeed_mmbi_channel));

	chan->bmc_int_en = true;
	/* H2B Interrupt */
	ret = of_property_read_u8(dev->of_node, "bmc-int-value", &chan->bmc_int_value);
	if (ret) {
		dev_err(dev, "cannot get valid MMBI H2B interrupt value\n");
		chan->bmc_int_en = false;
	}
	ret = of_property_read_u32(dev->of_node, "bmc-int-location", &chan->bmc_int_location);
	if (ret) {
		dev_err(dev, "cannot get valid MMBI H2B interrupt location\n");
		chan->bmc_int_en = false;
	}
	/* B2H Interrupt */
	chan->host_int_en = true;
	ret = of_property_read_u8(dev->of_node, "msi", &chan->host_int_value);
	if (ret) {
		dev_err(dev, "cannot get valid MMBI B2H interrupt location\n");
		chan->host_int_en = false;
	}

	ret = mmbi->platform->mmbi_init(pdev);
	if (ret) {
		dev_err(dev, "Initialize pcie mmbi failed\n");
		goto out_irq;
	}

	dev_info(dev, "ASPEED PCIe MMBI Dev %d: driver successfully loaded.\n", mmbi->id);

	return 0;
out_irq:
	devm_free_irq(dev, mmbi->irq, mmbi);
out_unmap:
	iounmap(mmbi->mem_virt);
out_region:
	devm_kfree(dev, mmbi);
	dev_warn(dev, "aspeed bmc device: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static void aspeed_pcie_mmbi_remove(struct platform_device *pdev)
{
	struct aspeed_pcie_mmbi *mmbi = platform_get_drvdata(pdev);

	cancel_work_sync(&mmbi->chan.work);
	unregister_netdev(mmbi->chan.ndev);
	devm_free_irq(&pdev->dev, mmbi->irq, mmbi);
	iounmap(mmbi->mem_virt);
	devm_kfree(&pdev->dev, mmbi);
}

static struct platform_driver aspeed_pcie_mmbi_driver = {
	.probe		= aspeed_pcie_mmbi_probe,
	.remove		= aspeed_pcie_mmbi_remove,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_pcie_mmbi_of_matches,
	},
};

module_platform_driver(aspeed_pcie_mmbi_driver);

MODULE_AUTHOR("Jacky Chou <jacky_chou@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PCI-E MMBI Driver");
MODULE_LICENSE("GPL");
