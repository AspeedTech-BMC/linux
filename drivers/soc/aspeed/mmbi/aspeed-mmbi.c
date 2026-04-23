// SPDX-License-Identifier: GPL-2.0+
/*
 * ASPEED MMBI (Memory Mapped Bus Interface) Protocol Driver
 *
 * Copyright (c) 2026 ASPEED Technology Inc.
 */

#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "aspeed-mmbi.h"
#include "aspeed-mmbi-internal.h"
#include "aspeed-mmbi-bmc.h"
#include "aspeed-mmbi-host.h"

static DEFINE_IDA(mmbi_ida);

static void mmbi_update_rd_ptr(u8 __iomem *read_ptr, u32 read_size, u32 read_buf_size)
{
	u32 reg, low, addr, reg_rb;
	u32 retry_cnt;
	if (!read_size || (read_size & 3)) {
		pr_err("%s: read_size must be non-zero and multiple of 4\n", __func__);
		return;
	}

	read_ptr += sizeof(u32); /* read pointer is located at offset 4 in the structure */
	reg = ioread32(read_ptr);

	low = reg & ~MMBI_PTR_ADDR_MASK; /* bits [1:0] */
	addr = reg & MMBI_PTR_ADDR_MASK; /* bits [31:2] */

	addr = (addr + read_size) % read_buf_size;
	addr &= MMBI_PTR_ADDR_MASK;
	retry_cnt = 0;
	iowrite32(addr | low, read_ptr);
	do {
		reg_rb = ioread32(read_ptr);
		retry_cnt++;
		if (retry_cnt > 5) {
			pr_info("%s: read pointer update not reflected after %u retries, reg 0x%x, expected 0x%x\n",
				__func__, retry_cnt, reg_rb, addr | low);
			break;
		}
	} while (reg_rb != (addr | low));
}

static void mmbi_update_wr_ptr(u8 __iomem *write_ptr, u32 write_size, u32 write_buf_size)
{
	u32 reg, low, addr, reg_rb;
	u32 retry_cnt;
	if (!write_size || (write_size & 3)) {
		pr_err("%s: write_size must be non-zero and multiple of 4\n", __func__);
		return;
	}

	reg = ioread32(write_ptr);

	low = reg & ~MMBI_PTR_ADDR_MASK; /* bits [1:0] */
	addr = reg & MMBI_PTR_ADDR_MASK; /* bits [31:2] */

	addr = (addr + write_size) % write_buf_size;
	addr &= MMBI_PTR_ADDR_MASK;

	retry_cnt = 0;
	iowrite32(addr | low, write_ptr);
	do {
		reg_rb = ioread32(write_ptr);
		retry_cnt++;
		if (retry_cnt > 5) {
			pr_info("%s: write pointer update not reflected after %u retries, reg 0x%x, expected 0x%x\n",
				__func__, retry_cnt, reg_rb, addr | low);
			break;
		}
	} while (reg_rb != (addr | low));
}

static void mmbi_parse_hdr(u8 __iomem *buf_virt, u32 *payload_len, u8 *padding,
			   u8 *protocol)
{
	u8 hdr[MMBI_PKT_HDR_SIZE];
	u32 word24;
	int i;

	/* Read header bytes via ioread8() */
	for (i = 0; i < MMBI_PKT_HDR_SIZE; i++)
		hdr[i] = ioread8(buf_virt + i);

	/* Byte0..2 form the 24-bit field: [23:2]=len, [1:0]=pad */
	word24 = ((u32)hdr[0] << 16) | ((u32)hdr[1] << 8) | (u32)hdr[2];

	*padding = word24 & 0x3;
	*payload_len = (word24 >> 2) * 4 - *padding;
	*protocol = hdr[3] & MMBI_PKT_PROTOCOL_MASK;
}

static int mmbi_read(struct mmbi_chan_desc *chan, u8 *data, size_t read_len)
{
	struct mmbi_ins_desc *mmbi = chan->mmbi;
	u32 read_ptr, write_ptr, read_offset, buf_size, unhandled_len, base_addr;
	u32 data_len, pkt_len;
	u8 protocol, padding;
	size_t first_part;
	u8 __iomem *desc_virt;

	if (read_len == 0)
		return -EINVAL;

	if (mmbi->role == MMBI_ROLE_BMC) {
		read_ptr = chan->buffer_desc.h_ros_p;
		write_ptr = chan->buffer_desc.h_rws_p;
		buf_size = chan->h2b_l;
		base_addr = chan->h2b_ba_offset;
	} else {
		read_ptr = chan->buffer_desc.h_rws_p;
		write_ptr = chan->buffer_desc.h_ros_p;
		buf_size = chan->b2h_l;
		base_addr = chan->b2h_ba_offset;
	}

	unhandled_len = mmbi_channel_unhandled_length(mmbi->desc_virt + read_ptr,
						      mmbi->desc_virt + write_ptr,
								      buf_size);

	if (unhandled_len == 0)
		return 0;

	desc_virt = mmbi->desc_virt + base_addr;
	data_len = 0;
	read_offset = MMBI_PTR_ADDR_ALIGN(be32_to_cpu(ioread32be(mmbi->desc_virt + read_ptr + sizeof(u32))));
	/* MMBI read/write has 4byte unit, so reading header will never have bundary issues */
	mmbi_parse_hdr(desc_virt + read_offset, &data_len, &padding, &protocol);

	if (data_len > read_len) {
		pr_warn("%s: not enough space(%ld) to read data(%u) for chan %u\n",
			__func__, read_len, data_len, chan->index);
		pr_warn("%s: read_offset %d padding %d protocol %d buf_size %d",
			__func__, read_offset, padding, protocol, buf_size);
		return -EINVAL;
	}
	pkt_len = MMBI_PKT_HDR_SIZE + data_len + padding;

	if (unhandled_len > pkt_len) {
		pr_warn("%s: more data unhandled than current packet length for chan %u, unhandled_len %u pkt_len %u\n",
			__func__, chan->index, unhandled_len, pkt_len);
	}

	/* move offset to payload */
	read_offset = (read_offset + MMBI_PKT_HDR_SIZE) % buf_size;
	if (protocol == MMBI_PROTOCOL_MCTP) {
		if (read_offset + data_len <= buf_size) {
			memcpy_fromio(data, desc_virt + read_offset, data_len);
		} else {
			first_part = buf_size - read_offset;
			memcpy_fromio(data,  desc_virt + read_offset, first_part);
			memcpy_fromio(data + first_part, desc_virt, data_len - first_part);
		}
	} else {
		pr_warn("%s: unsupported protocol %u on chan %u, discard packet\n",
			__func__, protocol, chan->index);
	}
	mmbi_update_rd_ptr(mmbi->desc_virt + read_ptr, pkt_len, buf_size);

	return protocol == MMBI_PROTOCOL_MCTP ? data_len : 0;
}

static int mmbi_write(struct mmbi_chan_desc *chan, const u8 *data, size_t data_len, u8 protocol)
{
	struct mmbi_ins_desc *mmbi = chan->mmbi;
	u32 read_ptr, write_ptr, write_offset, buf_size, avail_len, val, base_addr, total_len;
	size_t first_part;
	u8 padding;
	u8 __iomem *desc_virt;

	if (data_len == 0)
		return -EINVAL;

	if (!chan->priv->peer_ready) {
		pr_info("%s: chan %d peer not ready to receive data", __func__,
			chan->index);
		return -EAGAIN;
	}

	if (mmbi->role == MMBI_ROLE_BMC) {
		read_ptr = chan->buffer_desc.h_rws_p;
		write_ptr = chan->buffer_desc.h_ros_p;
		buf_size = chan->b2h_l;
		base_addr = chan->b2h_ba_offset;
	} else {
		read_ptr = chan->buffer_desc.h_ros_p;
		write_ptr = chan->buffer_desc.h_rws_p;
		buf_size = chan->h2b_l;
		base_addr = chan->h2b_ba_offset;
	}

	avail_len = mmbi_channel_avail_length(mmbi->desc_virt + read_ptr,
					      mmbi->desc_virt + write_ptr,
					      buf_size);

	total_len = data_len + ((protocol == MMBI_PROTOCOL_MCTP) ? 4 : 0);

	/* at least 8bytes for 4byte header and 4byte data */
	if (avail_len < MMBI_PKT_MIN_SIZE) {
		pr_info("%s: no available buffer to write for chan %u\n",
			 __func__, chan->index);
		return -EAGAIN;
	}

	if (total_len > avail_len) {
		pr_info("%s: no available buffer to write for chan %u, requested %d, avail %d\n",
			 __func__, chan->index, total_len, avail_len);
		return -EAGAIN;
	}

	write_offset = MMBI_PTR_ADDR_ALIGN(be32_to_cpu(ioread32be(mmbi->desc_virt + write_ptr)));
	desc_virt = mmbi->desc_virt + base_addr;
	/* Multi-protocol header */
	if (protocol == MMBI_PROTOCOL_MCTP) {
		padding = (4 - (data_len & 3)) & 3; /* Pad to 4 bytes */
		total_len += padding;
		val = (data_len + padding) >> 2;
		val &= 0x3FFFFF; /* 22 bits for length */
		iowrite8((val >> 14) & 0xff, desc_virt + write_offset);
		write_offset++;
		iowrite8((val >> 6) & 0xff, desc_virt + write_offset);
		write_offset++;
		iowrite8(((val << 2) & 0xff) | padding, desc_virt + write_offset);
		write_offset++;
		iowrite8(MMBI_PKT_PROTOCOL_MASK & protocol, desc_virt + write_offset);
		write_offset++;
	}

	write_offset %= buf_size;
	if (write_offset + data_len < buf_size) {
		memcpy_toio(desc_virt + write_offset, data, data_len);
	} else {
		first_part = buf_size - write_offset;
		memcpy_toio(desc_virt + write_offset, data, first_part);
		memcpy_toio(desc_virt, data + first_part, data_len - first_part);
	}
	mmbi_update_wr_ptr(mmbi->desc_virt + write_ptr, total_len, buf_size);
	return data_len;
}

static int mmbi_misc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct mmbi_chan_priv *priv = container_of(miscdev, struct mmbi_chan_priv,
						      miscdev);
	struct mmbi_chan_desc *chan_desc = priv->chan;

	file->private_data = chan_desc;
	mmbi_channel_state_update(chan_desc, chan_desc->mmbi->desc_virt);

	if (!chan_desc->priv->running) {
		schedule_delayed_work(&chan_desc->priv->poll_work,
				      msecs_to_jiffies(chan_desc->poll_interval_ms));
		chan_desc->priv->running = true;
	}
	return 0;
}

static int mmbi_misc_release(struct inode *inode, struct file *file)
{
	struct mmbi_chan_desc *chan_desc = file->private_data;

	if (chan_desc->priv->running) {
		cancel_delayed_work_sync(&chan_desc->priv->poll_work);
		chan_desc->priv->running = false;
	}
	return 0;
}

static ssize_t mmbi_misc_read(struct file *file, char __user *buf, size_t count,
			      loff_t *ppos)
{
	struct mmbi_chan_desc *chan_desc = file->private_data;
	ssize_t ret;
	void *kbuf;

	if (count == 0)
		return 0;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	spin_lock(&chan_desc->priv->rx_lock);
	chan_desc->priv->rx_ready = false;
	ret = mmbi_read(chan_desc, kbuf, count);
	spin_unlock(&chan_desc->priv->rx_lock);
	if (ret > 0) {
		if (copy_to_user(buf, kbuf, ret)) {
			pr_err("%s: chan %u copy %zd bytes to user failed\n",
			       __func__, chan_desc->index, ret);
			ret = -EFAULT;
		}
	} else {
		pr_err("%s: read failed for chan %u ret %zd\n", __func__, chan_desc->index, ret);
	}
	kfree(kbuf);
	return ret;
}

static ssize_t mmbi_misc_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct mmbi_chan_desc *chan_desc = file->private_data;
	ssize_t ret;
	void *kbuf;

	if (count == 0)
		return 0;

	kbuf = memdup_user(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	/* currently only MCTP defined in the spec */
	spin_lock(&chan_desc->priv->tx_lock);
	ret = mmbi_write(chan_desc, kbuf, count, MMBI_PROTOCOL_MCTP);
	spin_unlock(&chan_desc->priv->tx_lock);
	kfree(kbuf);
	return ret;
}

static __poll_t mmbi_misc_poll(struct file *file, struct poll_table_struct *pt)
{
	struct mmbi_chan_desc *chan_desc = file->private_data;
	struct mmbi_ins_desc *mmbi = chan_desc->mmbi;
	__poll_t ret, requested_events;

	ret = 0;
	requested_events = poll_requested_events(pt);
	mmbi_channel_state_update(chan_desc, mmbi->desc_virt);

	if (chan_desc->priv->state != NORMAL_RUNTIME && chan_desc->priv->state != RESET_REQ_BY_BMC) {
		pr_info("%s: chan %u invalid state 0x%x, not ready for polling\n",
			 __func__, chan_desc->index, chan_desc->priv->state);
		return ret;
	}

	if (requested_events & POLL_IN)
		poll_wait(file, &chan_desc->priv->rx_wait, pt);
	if (requested_events & POLL_OUT)
		poll_wait(file, &chan_desc->priv->tx_wait, pt);

	if (chan_desc->priv->rx_ready)
		ret |= POLL_IN | POLLRDNORM;
	if (chan_desc->priv->tx_ready)
		ret |= POLL_OUT | POLLWRNORM;

	return ret;
}

static const struct file_operations mmbi_fops = {
	.owner = THIS_MODULE,
	.open = mmbi_misc_open,
	.read = mmbi_misc_read,
	.write = mmbi_misc_write,
	.poll = mmbi_misc_poll,
	.release = mmbi_misc_release,
};

static int mmbi_instance_init_miscdev(struct mmbi_ins_desc *mmbi)
{
	int ret, i;
	struct mmbi_chan_desc *chan_desc;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		chan_desc = &mmbi->chan_desc[i];

		chan_desc->priv->miscdev.name =
			devm_kasprintf(mmbi->dev, GFP_KERNEL, "mmbi%u-ch%u",
				       chan_desc->mmbi->ins_id, chan_desc->index);

		if (!chan_desc->priv || !chan_desc->priv->miscdev.name) {
			pr_err("%s: failed to allocate misc device name for chan %u\n",
				__func__, chan_desc->index);
			return -ENOMEM;
		}
		chan_desc->priv->miscdev.minor = MISC_DYNAMIC_MINOR;
		chan_desc->priv->miscdev.fops = &mmbi_fops;
		chan_desc->priv->miscdev.parent = mmbi->dev;

		ret = misc_register(&chan_desc->priv->miscdev);
		if (ret) {
			pr_err("%s: failed to register misc device for chan %u\n",
				__func__, chan_desc->index);
		}
	}
	return 0;
}

static void mmbi_channel_poll_handler(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct mmbi_chan_priv *priv = container_of(dwork, struct mmbi_chan_priv, poll_work);
	struct mmbi_chan_desc *chan_desc = priv->chan;
	struct mmbi_ins_desc *mmbi = chan_desc->mmbi;
	u8 __iomem *desc_virt = mmbi->desc_virt;

	u32 state_location, read_ptr, write_ptr;

	state_location = (mmbi->role == MMBI_ROLE_BMC) ? chan_desc->buffer_desc.h_rws_p :
					       chan_desc->buffer_desc.h_ros_p;

	write_ptr = ioread32(desc_virt + state_location);
	read_ptr = ioread32(desc_virt + state_location + sizeof(u32));

	if (write_ptr != chan_desc->priv->write_ptr || read_ptr != chan_desc->priv->read_ptr) {
		chan_desc->priv->write_ptr = write_ptr;
		chan_desc->priv->read_ptr = read_ptr;

		spin_lock(&chan_desc->priv->rx_lock);
		spin_lock(&chan_desc->priv->tx_lock);
		if (mmbi->role == MMBI_ROLE_BMC)
			mmbi_channel_irq_bmc(chan_desc);
		else
			mmbi_channel_irq_host(chan_desc);

		spin_unlock(&chan_desc->priv->tx_lock);
		spin_unlock(&chan_desc->priv->rx_lock);
	}

	/* Reschedule the work */
	schedule_delayed_work(&chan_desc->priv->poll_work, msecs_to_jiffies(chan_desc->poll_interval_ms));
}

static int mmbi_instance_init_channel_priv(struct mmbi_ins_desc *mmbi)
{
	int i;
	struct mmbi_chan_desc *chan_desc;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		chan_desc = &mmbi->chan_desc[i];
		INIT_DELAYED_WORK(&chan_desc->priv->poll_work, mmbi_channel_poll_handler);
		spin_lock_init(&chan_desc->priv->rx_lock);
		spin_lock_init(&chan_desc->priv->tx_lock);
		init_waitqueue_head(&chan_desc->priv->rx_wait);
		init_waitqueue_head(&chan_desc->priv->tx_wait);
		chan_desc->priv->tx_ready = true;
		chan_desc->priv->rx_ready = false;
		chan_desc->priv->read_ptr = 0;
		chan_desc->priv->write_ptr = 0;
		chan_desc->priv->running = false;
		if (chan_desc->poll_interval_ms == 0)
			chan_desc->poll_interval_ms = MMBI_POLL_INTERVAL_MS;
	}
	return 0;
}

int mmbi_channel_avail_length(u8 __iomem *read_structure,
			      u8 __iomem *write_structure, u32 buf_size)
{
	u32 __read_offset = ioread32be(read_structure + sizeof(u32));
	u32 __write_offset = ioread32be(write_structure);

	__read_offset = MMBI_PTR_ADDR_ALIGN(be32_to_cpu(__read_offset));
	__write_offset = MMBI_PTR_ADDR_ALIGN(be32_to_cpu(__write_offset));

	return (__write_offset >= __read_offset) ?
		       (buf_size - __write_offset + __read_offset) :
		       (__read_offset + __write_offset);
}
EXPORT_SYMBOL_GPL(mmbi_channel_avail_length);

int mmbi_channel_unhandled_length(u8 __iomem *read_structure,
				  u8 __iomem *write_structure, u32 buf_size)
{
	u32 __read_offset = ioread32be(read_structure + sizeof(u32));
	u32 __write_offset = ioread32be(write_structure);

	__read_offset = MMBI_PTR_ADDR_ALIGN(be32_to_cpu(__read_offset));
	__write_offset = MMBI_PTR_ADDR_ALIGN(be32_to_cpu(__write_offset));
	return (__write_offset >= __read_offset) ?
		       (__write_offset - __read_offset) :
		       (buf_size - __read_offset + __write_offset);
}
EXPORT_SYMBOL_GPL(mmbi_channel_unhandled_length);

void mmbi_channel_state_update(struct mmbi_chan_desc *chan, u8 __iomem *desc_virt)
{
	u8 __iomem *host_rws_vmem;
	u8 __iomem *host_ros_vmem;
	u8 host_rws_val;
	u8 host_ros_val;
	struct mmbi_buf_vpscb *buf_desc;
	enum mmbi_state prev_state;

	if (chan->buffer_type == MMBI_BUFFER_TYPE_VPSCB) {
		prev_state = chan->priv->state;
		buf_desc = &chan->buffer_desc;
		host_rws_vmem = desc_virt + buf_desc->h_rws_p;
		host_ros_vmem = desc_virt + buf_desc->h_ros_p;
		host_rws_val = ioread8(host_rws_vmem);
		host_ros_val = ioread8(host_ros_vmem);
		chan->priv->state = (MMBI_STATE_GET_IF_UP(host_ros_val) << 2) |
				   (MMBI_STATE_GET_RST(host_ros_val) << 2) |
				   MMBI_STATE_GET_IF_UP(host_rws_val) |
				   MMBI_STATE_GET_RST(host_rws_val);

		chan->priv->peer_ready = (chan->mmbi->role == MMBI_ROLE_BMC) ?
						mmbi_get_ready(host_rws_vmem) :
						mmbi_get_ready(host_ros_vmem);
	} else {
		pr_err("\t%s: unsupported buffer type %d\n",
			__func__, chan->buffer_type);
		chan->priv->state = POWER_UP_OR_ERROR;
	}
}
EXPORT_SYMBOL_GPL(mmbi_channel_state_update);

int mmbi_instance_init(struct mmbi_ins_desc *mmbi)
{
	int rc, i;

	if (!mmbi) {
		rc = -EINVAL;
		goto out_fail;
	}

	if (mmbi->role == MMBI_ROLE_BMC) {
		if (mmbi->mmbi_version != MMBI_VERSION_1_0 &&
		    mmbi->mmbi_version != MMBI_VERSION_1_1) {
			rc = -EINVAL;
			goto out_fail;
		}
	}

	mmbi->ins_id = ida_alloc(&mmbi_ida, GFP_KERNEL);
	if (mmbi->ins_id < 0) {
		rc = mmbi->ins_id;
		goto out_fail;
	}

	for (i = 0; i < MMBI_MAX_CHANNELS; i++) {
		mmbi->chan_desc[i].priv = devm_kzalloc(mmbi->dev, sizeof(*mmbi->chan_desc[i].priv), GFP_KERNEL);
		if (!mmbi->chan_desc[i].priv) {
			rc = -ENOMEM;
			goto out_fail;
		}
		mmbi->chan_desc[i].priv->chan = &mmbi->chan_desc[i];
	}

	if (mmbi->role == MMBI_ROLE_BMC)
		rc = mmbi_instance_init_bmc(mmbi);
	else
		rc = mmbi_instance_init_host(mmbi);

	if (rc)
		goto ida_free;

	rc = mmbi_instance_init_miscdev(mmbi);
	if (rc)
		goto ida_free;

	rc = mmbi_instance_init_channel_priv(mmbi);
	if (rc)
		goto ida_free;

	return 0;

ida_free:
	ida_free(&mmbi_ida, mmbi->ins_id);
out_fail:
	pr_err("%s: failed to init MMBI instance\n",
		__func__);
	return rc;
}
EXPORT_SYMBOL_GPL(mmbi_instance_init);

void mmbi_instance_remove(struct mmbi_ins_desc *mmbi)
{
	int i;
	u8 __iomem *struct_desc;

	if (!mmbi)
		return;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		misc_deregister(&mmbi->chan_desc[i].priv->miscdev);
		struct_desc =
			mmbi->desc_virt +
			(mmbi->role == MMBI_ROLE_BMC ?
				 mmbi->chan_desc[i].buffer_desc.h_ros_p :
				 mmbi->chan_desc[i].buffer_desc.h_rws_p);
		mmbi_clr_ready(struct_desc);
		mmbi_clr_up(struct_desc);
	}

	ida_free(&mmbi_ida, mmbi->ins_id);
}
EXPORT_SYMBOL_GPL(mmbi_instance_remove);
