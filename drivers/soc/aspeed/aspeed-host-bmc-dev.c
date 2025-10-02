// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) ASPEED Technology Inc.

#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/poll.h>
#include <linux/bitfield.h>

#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/mctp.h>
#include <net/mctp.h>
#include <net/pkt_sched.h>

#include "aspeed-pcie-mmbi.h"

#define PCI_BMC_HOST2BMC_Q1		0x30000
#define PCI_BMC_HOST2BMC_Q2		0x30010
#define PCI_BMC_BMC2HOST_Q1		0x30020
#define PCI_BMC_BMC2HOST_Q2		0x30030
#define PCI_BMC_BMC2HOST_STS		0x30040
#define	 BMC2HOST_INT_STS_DOORBELL	BIT(31)
#define	 BMC2HOST_ENABLE_INTB		BIT(30)

#define	 BMC2HOST_Q1_FULL		BIT(27)
#define	 BMC2HOST_Q1_EMPTY		BIT(26)
#define	 BMC2HOST_Q2_FULL		BIT(25)
#define	 BMC2HOST_Q2_EMPTY		BIT(24)
#define	 BMC2HOST_Q1_FULL_UNMASK	BIT(23)
#define	 BMC2HOST_Q1_EMPTY_UNMASK	BIT(22)
#define	 BMC2HOST_Q2_FULL_UNMASK	BIT(21)
#define	 BMC2HOST_Q2_EMPTY_UNMASK	BIT(20)

#define PCI_BMC_HOST2BMC_STS		0x30044
#define	 HOST2BMC_INT_STS_DOORBELL	BIT(31)
#define	 HOST2BMC_ENABLE_INTB		BIT(30)

#define	 HOST2BMC_Q1_FULL		BIT(27)
#define	 HOST2BMC_Q1_EMPTY		BIT(26)
#define	 HOST2BMC_Q2_FULL		BIT(25)
#define	 HOST2BMC_Q2_EMPTY		BIT(24)
#define	 HOST2BMC_Q1_FULL_UNMASK	BIT(23)
#define	 HOST2BMC_Q1_EMPTY_UNMASK	BIT(22)
#define	 HOST2BMC_Q2_FULL_UNMASK	BIT(21)
#define	 HOST2BMC_Q2_EMPTY_UNMASK	BIT(20)

static DEFINE_IDA(bmc_device_ida);

#define MMBI_MAX_INST		6
#define VUART_MAX_PARMS		2
#define ASPEED_QUEUE_NUM	2
#define MAX_MSI_NUM		8

enum aspeed_platform_id {
	ASPEED,
	ASPEED_AST2700_SOC1,
};

enum queue_index {
	QUEUE1 = 0,
	QUEUE2,
};

enum msi_index {
	BMC_MSI,
	MBX_MSI,
	VUART0_MSI,
	VUART1_MSI,
	MMBI0_MSI,
	MMBI1_MSI,
	MMBI2_MSI,
	MMBI3_MSI,
};

/* Match msi_index */
static int ast2600_msi_idx_table[MAX_MSI_NUM] = { 4, 21, 16, 15 };
static int ast2700_soc0_msi_idx_table[MAX_MSI_NUM] = { 0, 11, 6, 5, 28, 29, 30, 31 };
/* ARRAY = MMIB0_MSI, MMBI1_MSI, MMBI2_MSI, MMBI3_MSI, MMBI4_MSI, MMBI5_MSI */
static int ast2700_soc1_msi_idx_table[MAX_MSI_NUM] = { 1, 2, 3, 4, 5, 6 };

struct aspeed_platform {
	int (*setup)(struct pci_dev *pdev);
};

struct aspeed_queue_message {
	/* Queue waiters for idle engine */
	wait_queue_head_t tx_wait;
	wait_queue_head_t rx_wait;
	struct kernfs_node *kn;
	struct bin_attribute bin;
	int index;
	struct aspeed_pci_bmc_dev *pci_bmc_device;
};

struct aspeed_pcie_mmbi {
	resource_size_t base;
	resource_size_t mem_size;
	void __iomem *mem;
	u32 segment_size;
	int irq;
	int id;
	struct aspeed_mmbi_channel chan;
	const char *dev_name;
};

struct aspeed_pci_bmc_dev {
	struct device *dev;
	struct miscdevice miscdev;
	struct aspeed_platform *platform;
	kernel_ulong_t driver_data;
	int id;

	unsigned long mem_bar_base;
	unsigned long mem_bar_size;
	void __iomem *mem_bar_reg;

	unsigned long message_bar_base;
	unsigned long message_bar_size;
	void __iomem *msg_bar_reg;

	void __iomem *pcie_sio_decode_addr;

	struct aspeed_queue_message queue[ASPEED_QUEUE_NUM];

	void __iomem *sio_mbox_reg;
	struct uart_8250_port uart[VUART_MAX_PARMS];
	int uart_line[VUART_MAX_PARMS];

	/* Interrupt
	 * The index of array is using to enum msi_index
	 */
	int *msi_idx_table;

	bool ast2700_soc1;

	/* AST2700 MMBI */
	struct aspeed_pcie_mmbi mmbi[MMBI_MAX_INST];
	int mmbi_start_msi;
};

#define PCIE_DEVICE_SIO_ADDR	(0x2E * 4)
#define BMC_MULTI_MSI		32

#define DRIVER_NAME "aspeed-host-bmc-dev"

static int mmbi_desc_init(struct aspeed_mmbi_channel *chan);

static u8 mmbi_get_bmc_rdy(struct aspeed_mmbi_channel *chan)
{
	struct host_ros hros;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));

	return hros.b_rdy;
}

static u8 mmbi_get_bmc_up(struct aspeed_mmbi_channel *chan)
{
	struct host_ros hros;

	memcpy_fromio(&hros, chan->hros_vmem, sizeof(hros));

	return hros.b_up;
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

static u8 mmbi_get_host_up(struct aspeed_mmbi_channel *chan)
{
	struct host_rws hrws;

	memcpy_fromio(&hrws, chan->hrws_vmem, sizeof(hrws));

	return hrws.h_up;
}

static void mmbi_set_host_rst(struct aspeed_mmbi_channel *chan, bool set)
{
	struct host_rws hrws;

	memcpy_fromio(&hrws, chan->hrws_vmem, sizeof(hrws));
	hrws.h_rst = set;
	memcpy_toio(chan->hrws_vmem, &hrws, sizeof(hrws));
}

static void mmbi_set_host_rdy(struct aspeed_mmbi_channel *chan, bool set)
{
	struct host_rws hrws;

	memcpy_fromio(&hrws, chan->hrws_vmem, sizeof(hrws));
	hrws.h_rdy = set;
	memcpy_toio(chan->hrws_vmem, &hrws, sizeof(hrws));
}

static void mmbi_set_host_up(struct aspeed_mmbi_channel *chan, bool set)
{
	struct host_rws hrws;

	memcpy_fromio(&hrws, chan->hrws_vmem, sizeof(hrws));
	hrws.h_up = set;
	memcpy_toio(chan->hrws_vmem, &hrws, sizeof(hrws));
}

static void get_h2b_avail_buf_len(struct aspeed_mmbi_channel *chan, ssize_t *avail_buf_len)
{
	struct device *dev = chan->dev;
	u32 h2b_rp, h2b_wp;

	h2b_rp = GET_H2B_READ_POINTER(chan);
	h2b_wp = GET_H2B_WRITE_POINTER(chan);
	dev_dbg(dev, "MMBI HRWS - h2b_rp: 0x%0x, h2b_wp: 0x%0x\n", h2b_rp, h2b_wp);

	if (h2b_wp >= h2b_rp)
		*avail_buf_len = chan->h2b_cb_size - h2b_wp + h2b_rp;
	else
		*avail_buf_len = h2b_rp - h2b_wp;
}

static u8 mmbi_get_state(struct aspeed_mmbi_channel *chan)
{
	u8 state = 0;

	state = mmbi_get_bmc_up(chan) << 3;
	state |= mmbi_get_bmc_rst(chan) << 2;
	state |= mmbi_get_host_up(chan) << 1;
	state |= mmbi_get_host_rst(chan);

	dev_dbg(chan->dev, "MMBI state: 0x%x\n", state);

	return state;
}

static void raise_h2b_interrupt(struct aspeed_mmbi_channel *chan)
{
	if (!chan->bmc_int_en)
		return;

	writeb(chan->bmc_int_value, chan->desc_vmem + chan->bmc_int_location);
}

static int mmbi_state_check(struct aspeed_mmbi_channel *chan)
{
	enum mmbi_state current_state = mmbi_get_state(chan);
	struct device *dev = chan->dev;
	int ret;

	switch (current_state) {
	case INIT_COMPLETED:
		dev_dbg(dev, "Get INIT_COMPLETED state from BMC");

		ret = mmbi_desc_init(chan);
		if (ret) {
			dev_warn(dev, "Check MMBI signature timeout\n");
			raise_h2b_interrupt(chan);
		}
		return 1;
	case RESET_REQ_BY_BMC:
		dev_dbg(dev, "Get RESET_REQ_BY_BMC state from BMC");

		/* Change state to RESET_ACKED */
		mmbi_set_host_rst(chan, 1);

		dev_dbg(dev, "Change state to RESET_ACKED to BMC");
		raise_h2b_interrupt(chan);
	default:
		break;
	}

	return 0;
}

static int mmbi_bmc_up_check(struct aspeed_mmbi_channel *chan)
{
	u64 __timeout_us = 1000;
	ktime_t __timeout = ktime_add_us(ktime_get(), __timeout_us);

	for (;;) {
		enum mmbi_state current_state = mmbi_get_state(chan);

		if (current_state == INIT_COMPLETED)
			break;
		if (__timeout_us && ktime_compare(ktime_get(), __timeout) > 0)
			return -EAGAIN;
	}

	return 0;
}

static void update_host_rws(struct aspeed_mmbi_channel *chan, unsigned int w_len,
			    unsigned int r_len)
{
	struct device *dev = chan->dev;
	struct host_rws hrws;
	u32 h2b_wp, b2h_rp;

	h2b_wp = GET_H2B_WRITE_POINTER(chan);
	b2h_rp = GET_B2H_READ_POINTER(chan);

	dev_dbg(dev, "MMBI HRWS - b2h_rp: 0x%0x, h2b_wp: 0x%0x\n", b2h_rp, h2b_wp);

	/* Advance the H2B CB offset for next write */
	if ((h2b_wp + w_len) <= chan->h2b_cb_size)
		h2b_wp += w_len;
	else
		h2b_wp = h2b_wp + w_len - chan->h2b_cb_size;

	/* Advance the B2H CB offset till where BMC read data */
	if ((b2h_rp + r_len) <= chan->b2h_cb_size)
		b2h_rp += r_len;
	else
		b2h_rp = b2h_rp + r_len - chan->b2h_cb_size;

	memcpy_fromio(&hrws, chan->hrws_vmem, sizeof(hrws));

	hrws.h2b_wp = FIELD_GET(H2B_WRITE_POINTER_MASK, h2b_wp);
	hrws.b2h_rp = FIELD_GET(B2H_READ_POINTER_MASK, b2h_rp);
	memcpy_toio(chan->hrws_vmem, &hrws, sizeof(hrws));
	dev_dbg(dev, "Updating HRWS - b2h_rp: 0x%0x, h2b_wp: 0x%0x\n", b2h_rp, h2b_wp);

	if (w_len != 0)
		raise_h2b_interrupt(chan);
}

static int get_mmbi_header(struct aspeed_mmbi_channel *chan, u32 *data_length, u8 *type,
			   u32 *unread_data_len, u8 *padding)
{
	u32 h2b_wp, h2b_rp, b2h_wp, b2h_rp;
	struct mmbi_header header;

	h2b_wp = GET_H2B_WRITE_POINTER(chan);
	h2b_rp = GET_H2B_READ_POINTER(chan);
	b2h_wp = GET_B2H_WRITE_POINTER(chan);
	b2h_rp = GET_B2H_READ_POINTER(chan);
	dev_dbg(chan->dev, "MMBI HRWS - h2b_wp: 0x%0x, b2h_rp: 0x%0x\n", h2b_wp, b2h_rp);
	dev_dbg(chan->dev, "MMBI HROS - b2h_wp: 0x%0x, h2b_rp: 0x%0x\n", b2h_wp, h2b_rp);

	if (b2h_wp >= b2h_rp)
		*unread_data_len = b2h_wp - b2h_rp;
	else
		*unread_data_len = chan->b2h_cb_size - b2h_rp + b2h_wp;

	if (*unread_data_len < sizeof(struct mmbi_header)) {
		dev_dbg(chan->dev, "No data to read(%d - %d)\n", b2h_wp, b2h_rp);
		return -EAGAIN;
	}

	dev_dbg(chan->dev, "READ MMBI header from: %p\n", chan->b2h_cb_vmem + b2h_rp);

	/* Extract MMBI protocol - protocol type and length */
	if ((b2h_rp + sizeof(header)) <= chan->b2h_cb_size) {
		memcpy_fromio(&header, chan->b2h_cb_vmem + b2h_rp, sizeof(header));
	} else {
		ssize_t chunk_len = chan->b2h_cb_size - b2h_rp;

		memcpy_fromio(&header, chan->b2h_cb_vmem + b2h_rp, chunk_len);
		memcpy_fromio(((u8 *)&header) + chunk_len, chan->b2h_cb_vmem,
			      sizeof(header) - chunk_len);
	}

	*data_length = (header.pkt_len << 2) - sizeof(header) - header.pkt_pad;
	*padding = header.pkt_pad;
	*type = header.pkt_type;

	return 0;
}

static int aspeed_mmbi_write(struct aspeed_mmbi_channel *chan, const char *buffer, size_t len,
			     protocol_type type)
{
	struct device *dev = chan->dev;
	struct mmbi_header header = {0};
	ssize_t avail_buf_len;
	ssize_t total_len;
	ssize_t wt_offset;
	ssize_t chunk_len;
	ssize_t end_offset;
	u8 padding = 0;

	/* If BMC READY bit is not set, Just discard the write. */
	if (!GET_BMC_READY_BIT(chan)) {
		dev_dbg(dev, "Host not ready, discarding request...\n");
		return -EAGAIN;
	}

	get_h2b_avail_buf_len(chan, &avail_buf_len);

	dev_dbg(dev, "H2B buffer empty space: %zd\n", avail_buf_len);

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

	wt_offset = GET_H2B_WRITE_POINTER(chan);
	end_offset = chan->h2b_cb_size;

	/* Copy Header */
	if ((end_offset - wt_offset) >= sizeof(header)) {
		memcpy_toio(chan->h2b_cb_vmem + wt_offset, &header, sizeof(header));
		wt_offset += sizeof(header);
	} else {
		chunk_len = end_offset - wt_offset;
		dev_dbg(dev, "Write header chunk_len: %zd\n", chunk_len);
		memcpy_toio(chan->h2b_cb_vmem + wt_offset, &header, chunk_len);
		memcpy_toio(chan->h2b_cb_vmem, (u8 *)&header + chunk_len,
			    (sizeof(header) - chunk_len));
		wt_offset = (sizeof(header) - chunk_len);
	}

	/* Write the data */
	if ((end_offset - wt_offset) >= len) {
		memcpy_toio(&chan->h2b_cb_vmem[wt_offset], buffer, len);
		wt_offset += len;
	} else {
		chunk_len = end_offset - wt_offset;
		dev_dbg(dev, "Write data chunk_len: %zd\n", chunk_len);
		memcpy_toio(&chan->h2b_cb_vmem[wt_offset], buffer, chunk_len);
		wt_offset = 0;
		memcpy_toio(&chan->h2b_cb_vmem[wt_offset], buffer + chunk_len, len - chunk_len);
		wt_offset += len - chunk_len;
	}

	update_host_rws(chan, total_len, 0);

	return 0;
}

static void aspeed_mmbi_read(struct aspeed_mmbi_channel *chan, char *buffer, size_t len, u8 padding)
{
	struct device *dev = chan->dev;
	ssize_t rd_offset;
	u32 b2h_rp;

	b2h_rp = GET_B2H_READ_POINTER(chan);
	if ((b2h_rp + sizeof(struct mmbi_header)) <= chan->b2h_cb_size)
		rd_offset = b2h_rp + sizeof(struct mmbi_header);
	else
		rd_offset = b2h_rp + sizeof(struct mmbi_header) - chan->b2h_cb_size;

	/* Extract data and copy to user space application */
	dev_dbg(dev, "READ MMBI Data from: %p and length: %zd\n",
		chan->b2h_cb_vmem + rd_offset, len);

	if ((chan->b2h_cb_size - rd_offset) >= len) {
		memcpy_fromio(buffer, chan->b2h_cb_vmem + rd_offset, len);
		rd_offset += len;
	} else {
		ssize_t chunk_len;

		chunk_len = chan->b2h_cb_size - rd_offset;
		dev_dbg(dev, "Read data chunk_len: %zd\n", chunk_len);
		memcpy_fromio(buffer, chan->b2h_cb_vmem + rd_offset, chunk_len);

		rd_offset = 0;
		memcpy_fromio(buffer + chunk_len, chan->b2h_cb_vmem + rd_offset,
			      len - chunk_len);
	}

	update_host_rws(chan, 0, len + sizeof(struct mmbi_header) + padding);
}

static void mctp_mmbi_rx(struct aspeed_mmbi_channel *chan)
{
	struct net_device *ndev;
	struct sk_buff *skb;
	struct mctp_skb_cb *cb;
	u32 req_data_len, unread_data_len;
	u8 type, padding;
	int status;

	if (get_mmbi_header(chan, &req_data_len, &type, &unread_data_len, &padding) != 0)
		return;

	dev_dbg(chan->dev, "%s: Length: 0x%0x, Protocol Type: %d, Unread data: %d\n", __func__,
		req_data_len, type, unread_data_len);

	ndev = chan->ndev;

	skb = netdev_alloc_skb(ndev, req_data_len);
	if (!skb) {
		ndev->stats.rx_dropped++;
		update_host_rws(chan, 0, req_data_len + sizeof(struct mmbi_header));
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

	if (!mmbi_get_bmc_rdy(&mctp->mmbi->chan) || skb->len > MCTP_MMBI_MTU_MAX) {
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

	snprintf(name, sizeof(name), "mctpmmbi%d", chan->mmbi->id);
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

static struct aspeed_pci_bmc_dev *file_aspeed_bmc_device(struct file *file)
{
	return container_of(file->private_data, struct aspeed_pci_bmc_dev, miscdev);
}

static int aspeed_pci_bmc_dev_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = file_aspeed_bmc_device(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	pgprot_t prot = vma->vm_page_prot;

	if (vma->vm_pgoff + vsize > pci_bmc_dev->mem_bar_base + 0x100000)
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (pci_bmc_dev->mem_bar_base >> PAGE_SHIFT) + vma->vm_pgoff,
			    vsize, prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations aspeed_pci_bmc_dev_fops = {
	.owner		= THIS_MODULE,
	.mmap		= aspeed_pci_bmc_dev_mmap,
};

static ssize_t aspeed_queue_rx(struct file *filp, struct kobject *kobj, struct bin_attribute *attr,
			       char *buf, loff_t off, size_t count)
{
	struct aspeed_queue_message *queue = attr->private;
	struct aspeed_pci_bmc_dev *pci_bmc_device = queue->pci_bmc_device;
	int index = queue->index;
	u32 *data = (u32 *)buf;
	int ret;

	ret = wait_event_interruptible(queue->rx_wait,
				       !(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS) &
				       ((index == QUEUE1) ? BMC2HOST_Q1_EMPTY : BMC2HOST_Q2_EMPTY)));
	if (ret)
		return -EINTR;

	data[0] = readl(pci_bmc_device->msg_bar_reg +
			((index == QUEUE1) ? PCI_BMC_BMC2HOST_Q1 : PCI_BMC_BMC2HOST_Q2));

	writel(HOST2BMC_INT_STS_DOORBELL | HOST2BMC_ENABLE_INTB,
	       pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS);

	return sizeof(u32);
}

static ssize_t aspeed_queue_tx(struct file *filp, struct kobject *kobj, struct bin_attribute *attr,
			       char *buf, loff_t off, size_t count)
{
	struct aspeed_queue_message *queue = attr->private;
	struct aspeed_pci_bmc_dev *pci_bmc_device = queue->pci_bmc_device;
	int index = queue->index;
	u32 tx_buff;
	int ret;

	if (count != sizeof(u32))
		return -EINVAL;

	ret = wait_event_interruptible(queue->tx_wait,
				       !(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS) &
				       ((index == QUEUE1) ? HOST2BMC_Q1_FULL : HOST2BMC_Q2_FULL)));
	if (ret)
		return -EINTR;

	memcpy(&tx_buff, buf, 4);
	writel(tx_buff, pci_bmc_device->msg_bar_reg +
				((index == QUEUE1) ? PCI_BMC_HOST2BMC_Q1 : PCI_BMC_HOST2BMC_Q2));
	//trigger to host
	writel(HOST2BMC_INT_STS_DOORBELL | HOST2BMC_ENABLE_INTB,
	       pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS);

	return sizeof(u32);
}

static irqreturn_t aspeed_pci_host_bmc_device_interrupt(int irq, void *dev_id)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_id;
	u32 bmc2host_q_sts = readl(pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_INT_STS_DOORBELL)
		writel(BMC2HOST_INT_STS_DOORBELL,
		       pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_ENABLE_INTB)
		writel(BMC2HOST_ENABLE_INTB, pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_Q1_FULL)
		dev_info(pci_bmc_device->dev, "Q1 Full\n");

	if (bmc2host_q_sts & BMC2HOST_Q2_FULL)
		dev_info(pci_bmc_device->dev, "Q2 Full\n");

	//check q1
	if (!(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS) & HOST2BMC_Q1_FULL))
		wake_up_interruptible(&pci_bmc_device->queue[QUEUE1].tx_wait);

	if (!(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS) & BMC2HOST_Q1_EMPTY))
		wake_up_interruptible(&pci_bmc_device->queue[QUEUE1].rx_wait);
	//chech q2
	if (!(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS) & HOST2BMC_Q2_FULL))
		wake_up_interruptible(&pci_bmc_device->queue[QUEUE2].tx_wait);

	if (!(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS) & BMC2HOST_Q2_EMPTY))
		wake_up_interruptible(&pci_bmc_device->queue[QUEUE2].rx_wait);

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_pci_host_mbox_interrupt(int irq, void *dev_id)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_id;
	u32 isr = readl(pci_bmc_device->sio_mbox_reg + 0x94);

	if (isr & BIT(7))
		writel(BIT(7), pci_bmc_device->sio_mbox_reg + 0x94);

	return IRQ_HANDLED;
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
			update_host_rws(chan, 0, req_data_len + sizeof(struct mmbi_header) + padding);

		raise_h2b_interrupt(chan);
	}

	if (get_mmbi_header(chan, &req_data_len, &type, &unread_data_len, &padding) != 0)
		queue_work(system_unbound_wq, &chan->work);
}

static irqreturn_t aspeed_pci_mmbi_isr(int irq, void *dev_id)
{
	struct aspeed_pcie_mmbi *mmbi = dev_id;
	struct aspeed_mmbi_channel *chan = &mmbi->chan;
	ssize_t avail_buf_len;

	get_h2b_avail_buf_len(chan, &avail_buf_len);
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

static void aspeed_pci_setup_irq_resource(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);

	/* Assign static msi index table by platform */
	if (pdev->revision == 0x27) {
		if (pci_bmc_dev->driver_data == ASPEED) {
			pci_bmc_dev->msi_idx_table = ast2700_soc0_msi_idx_table;
		} else {
			pci_bmc_dev->msi_idx_table = ast2700_soc1_msi_idx_table;
			pci_bmc_dev->ast2700_soc1 = true;
		}
	} else {
		pci_bmc_dev->msi_idx_table = ast2600_msi_idx_table;
	}

	if (pci_alloc_irq_vectors(pdev, 1, BMC_MULTI_MSI, PCI_IRQ_INTX | PCI_IRQ_MSI) <= 1)
		/* Set all msi index to the first vector */
		memset(pci_bmc_dev->msi_idx_table, 0, sizeof(int) * MAX_MSI_NUM);
}

static int aspeed_pci_bmc_device_setup_queue(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret, i;

	for (i = 0; i < ASPEED_QUEUE_NUM; i++) {
		struct aspeed_queue_message *queue = &pci_bmc_device->queue[i];

		init_waitqueue_head(&queue->tx_wait);
		init_waitqueue_head(&queue->rx_wait);

		sysfs_bin_attr_init(&queue->bin);

		/* Queue name index starts from 1 */
		queue->bin.attr.name =
			devm_kasprintf(dev, GFP_KERNEL, "pci-bmc-dev-queue%d", (i + 1));
		queue->bin.attr.mode = 0600;
		queue->bin.read = aspeed_queue_rx;
		queue->bin.write = aspeed_queue_tx;
		queue->bin.size = 4;
		queue->bin.private = queue;

		ret = sysfs_create_bin_file(&pdev->dev.kobj, &queue->bin);
		if (ret) {
			dev_err(dev, "error for bin%d file\n", i);
			return ret;
		}

		queue->kn = kernfs_find_and_get(dev->kobj.sd, queue->bin.attr.name);
		if (!queue->kn) {
			sysfs_remove_bin_file(&dev->kobj, &queue->bin);
			return ret;
		}

		queue->index = i;
		queue->pci_bmc_device = pci_bmc_device;
	}

	return 0;
}

static int aspeed_pci_bmc_device_setup_vuart(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u16 vuart_ioport;
	int ret, i;

	for (i = 0; i < VUART_MAX_PARMS; i++) {
		/* Assign the line to non-exist device */
		pci_bmc_dev->uart_line[i] = -ENOENT;
		vuart_ioport = 0x3F8 - (i * 0x100);
		pci_bmc_dev->uart[i].port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
		pci_bmc_dev->uart[i].port.uartclk = 115200 * 16;
		pci_bmc_dev->uart[i].port.irq =
			pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[VUART0_MSI + i]);
		pci_bmc_dev->uart[i].port.dev = dev;
		pci_bmc_dev->uart[i].port.iotype = UPIO_MEM32;
		pci_bmc_dev->uart[i].port.iobase = 0;
		pci_bmc_dev->uart[i].port.mapbase =
			pci_bmc_dev->message_bar_base + (vuart_ioport << 2);
		pci_bmc_dev->uart[i].port.membase = 0;
		pci_bmc_dev->uart[i].port.type = PORT_16550A;
		pci_bmc_dev->uart[i].port.flags |= (UPF_IOREMAP | UPF_FIXED_PORT | UPF_FIXED_TYPE);
		pci_bmc_dev->uart[i].port.regshift = 2;
		ret = serial8250_register_8250_port(&pci_bmc_dev->uart[i]);
		if (ret < 0) {
			dev_err_probe(dev, ret, "Can't setup PCIe VUART\n");
			return ret;
		}
		pci_bmc_dev->uart_line[i] = ret;
	}
	return 0;
}

static int aspeed_pci_bmc_device_setup_memory_mapping(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	pci_bmc_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	pci_bmc_dev->miscdev.name =
		devm_kasprintf(dev, GFP_KERNEL, "%s%d", DRIVER_NAME, pci_bmc_dev->id);
	pci_bmc_dev->miscdev.fops = &aspeed_pci_bmc_dev_fops;
	pci_bmc_dev->miscdev.parent = dev;

	ret = misc_register(&pci_bmc_dev->miscdev);
	if (ret) {
		pr_err("host bmc register fail %d\n", ret);
		return ret;
	}

	return 0;
}

static int aspeed_pci_bmc_device_setup_mbox(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	/* setup mbox */
	pci_bmc_dev->pcie_sio_decode_addr = pci_bmc_dev->msg_bar_reg + PCIE_DEVICE_SIO_ADDR;
	writel(0xaa, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0xa5, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0xa5, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x07, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x0e, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	/* disable */
	writel(0x30, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x00, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	/* set decode address 0x100 */
	writel(0x60, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x01, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	writel(0x61, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x00, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	/* enable */
	writel(0x30, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x01, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	pci_bmc_dev->sio_mbox_reg = pci_bmc_dev->msg_bar_reg + 0x400;

	ret = devm_request_irq(dev,
			       pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[MBX_MSI]),
			       aspeed_pci_host_mbox_interrupt, IRQF_SHARED,
			       devm_kasprintf(dev, GFP_KERNEL, "aspeed-sio-mbox%d", pci_bmc_dev->id),
			       pci_bmc_dev);
	if (ret) {
		pr_err("host bmc device Unable to get IRQ %d\n", ret);
		return ret;
	}

	return 0;
}

static int mmbi_signature_check(struct aspeed_mmbi_channel *chan)
{
	u8 signature[6];
	u64 __timeout_us = 1000;
	ktime_t __timeout = ktime_add_us(ktime_get(), __timeout_us);

	for (;;) {
		memcpy_fromio(signature, chan->desc_vmem, 6);
		if (!memcmp(MMBI_SIGNATURE, signature, 6))
			break;
		if (__timeout_us && ktime_compare(ktime_get(), __timeout) > 0)
			return -ETIMEDOUT;
	}
	return 0;
}

static int mmbi_desc_init(struct aspeed_mmbi_channel *chan)
{
	struct aspeed_pcie_mmbi *mmbi = chan->mmbi;
	struct device *dev = chan->dev;
	struct mmbi_cap_desc desc;
	u8 __iomem *desc_base = chan->desc_vmem;
	int ret;

	/* First, check mmbi signature "#MMBI$" */
	ret = mmbi_signature_check(chan);
	if (ret) {
		dev_warn(dev, "Check MMBI signature timeout\n");
		return ret;
	}

	memcpy_fromio(&desc, chan->desc_vmem, sizeof(desc));

	/* HROS */
	if (((desc.bt_desc.h_ros_p << 3) + sizeof(struct host_ros)) >= mmbi->mem_size) {
		dev_warn(dev, "HROS is out of range");
		return -EINVAL;
	}
	chan->hros_vmem = desc_base + (desc.bt_desc.h_ros_p << 3);

	/* HRWS */
	if (((desc.bt_desc.h_rws_p << 3) + sizeof(struct host_rws)) >= mmbi->mem_size) {
		dev_warn(dev, "HRWS is out of range");
		return -EINVAL;
	}
	chan->hrws_vmem = desc_base + (desc.bt_desc.h_rws_p << 3);

	ret = mmbi_bmc_up_check(chan);
	if (ret) {
		dev_warn(dev, "Check BMC up timeout\n");
		return ret;
	}

	/* Implementations of MMBI described in this document shall indicate version 1 of MMBI */
	if (desc.version != 1) {
		dev_warn(dev, "MMBI version must be 1");
		goto err_mismatch;
	}

	/* This MMBI interface is intended for OS use */
	if (desc.os_use != 1) {
		dev_warn(dev, "This MMBI does not provide for OS");
		goto err_mismatch;
	}

	/* Current application is only MMBI Variable Packet Size Circular Buffers (VPSCB) v1 */
	if (desc.buffer_type != 1) {
		dev_warn(dev, "The buffer type is not VPSCB: (%d)", desc.buffer_type);
		goto err_mismatch;
	}

	/* B2H Buffer */
	if (((desc.b2h_ba << 3) + desc.b2h_l) > mmbi->mem_size) {
		dev_warn(dev, "B2H buffer is out of range");
		goto err_mismatch;
	}
	chan->b2h_cb_vmem = desc_base + (desc.b2h_ba << 3);
	chan->b2h_cb_size = desc.b2h_l;

	/* H2B Buffer */
	if (((desc.h2b_ba << 3) + desc.h2b_l) > mmbi->mem_size) {
		dev_warn(dev, "H2B buffer is out of range");
		goto err_mismatch;
	}
	chan->h2b_cb_vmem = desc_base + (desc.h2b_ba << 3);
	chan->h2b_cb_size = desc.h2b_l;

	dev_dbg(dev, "B2H mapped addr - desc: %p, hros: %p, b2h_cb: %p\n",
		chan->desc_vmem, chan->hros_vmem, chan->b2h_cb_vmem);
	dev_dbg(dev, "H2B mapped addr - hrws: %p, h2b_cb: %p\n", chan->hrws_vmem,
		chan->h2b_cb_vmem);

	dev_dbg(dev, "B2H buffer size: 0x%0x\n", chan->b2h_cb_size);
	dev_dbg(dev, "H2B buffer size: 0x%0x\n", chan->h2b_cb_size);

	/* Host Interrupt */
	chan->host_int_en = !!desc.bt_desc.h_int_t;
	if (chan->host_int_en) {
		/* 1 for PCIe */
		if (desc.bt_desc.h_int_t != 1)
			chan->host_int_en = 0;
		else
			chan->host_int_location = desc.bt_desc.h_int_l;
	}

	/* BMC Interrupt */
	chan->bmc_int_en = !!desc.bt_desc.bmc_int_t;
	if (chan->bmc_int_en) {
		if (desc.bt_desc.bmc_int_t != 1) {
			chan->bmc_int_en = 0;
		} else {
			chan->bmc_int_location = desc.bt_desc.bmc_int_l;
			chan->bmc_int_vmem = desc_base + chan->bmc_int_location;
			chan->bmc_int_value = desc.bt_desc.bmc_int_v;
		}
	}

	INIT_WORK(&chan->work, aspeed_mmbi_work_func);

	return 0;
err_mismatch:
	/* Change state to INIT_MISMATCH */
	mmbi_set_host_rst(chan, 1);
	return -EINVAL;
}

static int aspeed_pci_mmbi_init(struct aspeed_pcie_mmbi *mmbi)
{
	struct aspeed_mmbi_channel *chan = &mmbi->chan;
	struct device *dev = chan->dev;
	int ret;

	chan->desc_vmem = mmbi->mem;
	chan->mmbi = mmbi;

	ret = mmbi_desc_init(chan);
	if (ret) {
		dev_err(dev, "Unable to init mmbi desc\n");
		return ret;
	}

	/* Initialize MTCP function */
	ret = aspeed_mmbi_mctp_init(chan);
	if (ret) {
		dev_err(dev, "Unable to init mctp\n");
		return ret;
	}

	/* Change state to NORMAL_RUNTIME */
	mmbi_set_host_up(chan, 1);
	mmbi_set_host_rdy(chan, 1);
	/* Trigger BMC to finish normal runtime state */
	raise_h2b_interrupt(chan);

	return 0;
}

/* AST2700 PCIe MMBI
 * SoC : |  0          |  1                |
 * BAR : |  2  3  4  5 |  0  1  2  3  4  5 |
 * MMBI: |  0  1  2  3 |  0  1  2  3  4  5 |
 */
static void aspeed_pci_bmc_device_setup_mmbi(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct aspeed_pcie_mmbi *mmbi;
	u32 start_bar = 2, mmbi_max_inst = 4, start_msi = MMBI0_MSI;	/* AST2700 SoC0 */
	int i, rc = 0;

	/* AST2700 A1 supports MMBI */
	if (pdev->revision != 0x27)
		return;

	if (pci_bmc_dev->ast2700_soc1) {
		/* AST2700 SoC1 */
		start_bar = 0;
		mmbi_max_inst = 6;
		start_msi = 0;
	}

	for (i = 0; i < mmbi_max_inst; i++) {
		mmbi = &pci_bmc_dev->mmbi[i];

		/* Get MMBI BAR resource */
		mmbi->base = pci_resource_start(pdev, start_bar + i);
		mmbi->mem_size = pci_resource_len(pdev, start_bar + i);

		/* Check if there is bar */
		if (!mmbi->mem_size)
			continue;

		mmbi->mem = pci_ioremap_bar(pdev, start_bar + i);
		if (!mmbi->mem) {
			mmbi->mem_size = 0;
			continue;
		}

		mmbi->chan.dev = &pdev->dev;
		mmbi->dev_name = devm_kasprintf(mmbi->chan.dev, GFP_KERNEL, "pci-mmbi%d", i);
		mmbi->id = i;
		rc = aspeed_pci_mmbi_init(mmbi);
		if (rc < 0) {
			pr_err("Initialize MMBI device failed.\n");
			goto free_ioremap;
		}

		if (mmbi->chan.host_int_en) {
			mmbi->irq = pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[start_msi + i]);
			rc = devm_request_irq(&pdev->dev, mmbi->irq, aspeed_pci_mmbi_isr,
					      IRQF_SHARED, mmbi->dev_name, mmbi);
			if (rc) {
				pr_err("MMBI device %s unable to get IRQ %d\n", mmbi->dev_name, rc);
				mmbi->irq = 0;
				goto free_ioremap;
			}
		} else {
			mmbi->irq = 0;
		}
		continue;
free_ioremap:
		mmbi->mem_size = 0;
		pci_iounmap(pdev, mmbi->mem);
	}
}

static void aspeed_pci_host_bmc_device_release_queue(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	int i;

	for (i = 0; i < ASPEED_QUEUE_NUM; i++)
		sysfs_remove_bin_file(&pdev->dev.kobj, &pci_bmc_dev->queue[i].bin);
}

static void aspeed_pci_host_bmc_device_release_vuart(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	int i;

	for (i = 0; i < VUART_MAX_PARMS; i++) {
		if (pci_bmc_dev->uart_line[i] >= 0)
			serial8250_unregister_port(pci_bmc_dev->uart_line[i]);
	}
}

static void aspeed_pci_host_bmc_device_release_memory_mapping(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);

	if (!list_empty(&pci_bmc_dev->miscdev.list))
		misc_deregister(&pci_bmc_dev->miscdev);
}

static void aspeed_pci_release_mmbi(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct aspeed_pcie_mmbi *mmbi;
	int i;

	if (pdev->revision != 0x27)
		return;

	for (i = 0; i < MMBI_MAX_INST; i++) {
		mmbi = &pci_bmc_dev->mmbi[i];

		if (mmbi->mem_size == 0)
			continue;

		cancel_work_sync(&mmbi->chan.work);

		mmbi_set_host_rdy(&mmbi->chan, 0);
		mmbi_set_host_up(&mmbi->chan, 0);

		unregister_netdev(mmbi->chan.ndev);

		if (mmbi->mem)
			pci_iounmap(pdev, mmbi->mem);
		if (mmbi->irq != 0)
			devm_free_irq(&pdev->dev, mmbi->irq, mmbi);
	}
}

static int aspeed_pci_host_setup(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	int rc = 0;

	/* Get share memory BAR */
	pci_bmc_dev->mem_bar_base = pci_resource_start(pdev, 0);
	pci_bmc_dev->mem_bar_size = pci_resource_len(pdev, 0);
	pci_bmc_dev->mem_bar_reg = pci_ioremap_bar(pdev, 0);
	if (!pci_bmc_dev->mem_bar_reg)
		return -ENOMEM;

	/* Get Message BAR */
	pci_bmc_dev->message_bar_base = pci_resource_start(pdev, 1);
	pci_bmc_dev->message_bar_size = pci_resource_len(pdev, 1);
	pci_bmc_dev->msg_bar_reg = pci_ioremap_bar(pdev, 1);
	if (!pci_bmc_dev->msg_bar_reg) {
		rc = -ENOMEM;
		goto out_free0;
	}

	/* AST2600 ERRTA40: dummy read */
	if (pdev->revision < 0x27)
		(void)__raw_readl((void __iomem *)pci_bmc_dev->msg_bar_reg);

	rc = aspeed_pci_bmc_device_setup_queue(pdev);
	if (rc) {
		pr_err("Cannot setup Queue Message");
		goto out_free1;
	}

	rc = aspeed_pci_bmc_device_setup_memory_mapping(pdev);
	if (rc) {
		pr_err("Cannot setup Memory Mapping");
		goto out_free_queue;
	}

	rc = aspeed_pci_bmc_device_setup_mbox(pdev);
	if (rc) {
		pr_err("Cannot setup Mailnbox");
		goto out_free_mmapping;
	}

	rc = aspeed_pci_bmc_device_setup_vuart(pdev);
	if (rc) {
		pr_err("Cannot setup Virtual UART");
		goto out_free_mbox;
	}

	rc = devm_request_irq(&pdev->dev, pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[BMC_MSI]),
			      aspeed_pci_host_bmc_device_interrupt, IRQF_SHARED,
			      pci_bmc_dev->miscdev.name, pci_bmc_dev);
	if (rc) {
		pr_err("Get BMC DEVICE IRQ failed. (err=%d)\n", rc);
		goto out_free_uart;
	}

	/* Setup AST2700 PCIe MMBI device */
	aspeed_pci_bmc_device_setup_mmbi(pdev);

	return 0;

out_free_uart:
	aspeed_pci_host_bmc_device_release_vuart(pdev);
out_free_mbox:
	devm_free_irq(&pdev->dev, pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[MBX_MSI]),
		      pci_bmc_dev);
out_free_mmapping:
	aspeed_pci_host_bmc_device_release_memory_mapping(pdev);
out_free_queue:
	aspeed_pci_host_bmc_device_release_queue(pdev);
out_free1:
	pci_iounmap(pdev, pci_bmc_dev->msg_bar_reg);
out_free0:
	pci_iounmap(pdev, pci_bmc_dev->mem_bar_reg);

	pci_release_regions(pdev);
	return rc;
}

static int aspeed_pci_host_mmbi_device_setup(struct pci_dev *pdev)
{
	aspeed_pci_bmc_device_setup_mmbi(pdev);
	return 0;
}

static struct aspeed_platform aspeed_pcie_host[] = {
	{ .setup = aspeed_pci_host_setup },
	{ .setup = aspeed_pci_host_mmbi_device_setup },
	{ 0 }
};

static int aspeed_pci_host_bmc_device_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev;
	int rc = 0;

	pr_info("ASPEED BMC PCI ID %04x:%04x, IRQ=%u\n", pdev->vendor, pdev->device, pdev->irq);

	pci_bmc_dev = devm_kzalloc(&pdev->dev, sizeof(*pci_bmc_dev), GFP_KERNEL);
	if (!pci_bmc_dev)
		return -ENOMEM;

	/* Get platform id */
	pci_bmc_dev->driver_data = ent->driver_data;
	pci_bmc_dev->platform = &aspeed_pcie_host[ent->driver_data];

	pci_bmc_dev->id = ida_simple_get(&bmc_device_ida, 0, 0, GFP_KERNEL);
	if (pci_bmc_dev->id < 0)
		return pci_bmc_dev->id;

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "pci_enable_device() returned error %d\n", rc);
		return rc;
	}

	pci_set_master(pdev);
	pci_set_drvdata(pdev, pci_bmc_dev);

	/* Prepare IRQ resource */
	aspeed_pci_setup_irq_resource(pdev);

	/* Setup BMC PCI device */
	rc = pci_bmc_dev->platform->setup(pdev);
	if (rc) {
		dev_err(&pdev->dev, "ASPEED PCIe Host device returned error %d\n", rc);
		pci_free_irq_vectors(pdev);
		pci_disable_device(pdev);
		return rc;
	}

	return 0;
}

static void aspeed_pci_host_bmc_device_remove(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);

	if (pci_bmc_dev->driver_data == ASPEED) {
		aspeed_pci_host_bmc_device_release_queue(pdev);
		aspeed_pci_host_bmc_device_release_memory_mapping(pdev);
		aspeed_pci_host_bmc_device_release_vuart(pdev);

		devm_free_irq(&pdev->dev, pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[BMC_MSI]),
			      pci_bmc_dev);
		devm_free_irq(&pdev->dev, pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[MBX_MSI]),
			      pci_bmc_dev);
	}

	aspeed_pci_release_mmbi(pdev);

	ida_simple_remove(&bmc_device_ida, pci_bmc_dev->id);

	pci_iounmap(pdev, pci_bmc_dev->msg_bar_reg);
	pci_iounmap(pdev, pci_bmc_dev->mem_bar_reg);

	pci_free_irq_vectors(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

/**
 * This table holds the list of (VendorID,DeviceID) supported by this driver
 *
 */
static struct pci_device_id aspeed_host_bmc_dev_pci_ids[] = {
	/* ASPEED BMC Device */
	{ PCI_DEVICE(0x1A03, 0x2402), .class = 0xFF0000, .class_mask = 0xFFFF00,
	  .driver_data = ASPEED },
	/* AST2700 SoC1 MMBI device */
	{ PCI_DEVICE(0x1A03, 0x2402), .class = 0x0C0C00, .class_mask = (0xFFFF00),
	  .driver_data = ASPEED_AST2700_SOC1 },
	{
		0,
	}
};

MODULE_DEVICE_TABLE(pci, aspeed_host_bmc_dev_pci_ids);

static struct pci_driver aspeed_host_bmc_dev_driver = {
	.name		= DRIVER_NAME,
	.id_table	= aspeed_host_bmc_dev_pci_ids,
	.probe		= aspeed_pci_host_bmc_device_probe,
	.remove		= aspeed_pci_host_bmc_device_remove,
};

static int __init aspeed_host_bmc_device_init(void)
{
	return pci_register_driver(&aspeed_host_bmc_dev_driver);
}

static void aspeed_host_bmc_device_exit(void)
{
	/* unregister pci driver */
	pci_unregister_driver(&aspeed_host_bmc_dev_driver);
}

late_initcall(aspeed_host_bmc_device_init);
module_exit(aspeed_host_bmc_device_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Host BMC DEVICE Driver");
MODULE_LICENSE("GPL");
