// SPDX-License-Identifier: GPL-2.0+
/*
 * ASPEED MMBI (Memory Mapped Bus Interface) Protocol Driver
 *
 * Copyright (c) 2026 ASPEED Technology Inc.
 */

#include <linux/device.h>
#include <linux/dev_printk.h>
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

static void mmbi_instance_irq(struct work_struct *work)
{
	struct mmbi_ins_desc *mmbi = container_of(work, struct mmbi_ins_desc, work);

	/* Handle MMBI interrupt here */
	if (mmbi->role == MMBI_ROLE_BMC)
		mmbi_instance_irq_bmc(mmbi);
	else
		mmbi_instance_irq_host(mmbi);
}

static void mmbi_instance_pending_irq(struct work_struct *work)
{
	u32 val_location, location;
	struct mmbi_ins_desc *mmbi = container_of(to_delayed_work(work), struct mmbi_ins_desc, irq_pending_work);

	if (mmbi->pending_int) {
		if (mmbi->role == MMBI_ROLE_BMC) {
			val_location = MMBI_BMC_INT_VAL_OFFSET;
			location = mmbi->bmc_int_location;
		} else {
			val_location = MMBI_HOST_INT_VAL_OFFSET;
			location = mmbi->host_int_location;
		}
		mmbi_set_int_value(mmbi, val_location, location, 0);
	}
}

static int mmbi_read(struct mmbi_chan_desc *chan, u8 *data, size_t read_len)
{
	struct mmbi_ins_desc *mmbi = chan->mmbi;
	struct device *dev = chan->mmbi->dev;
	u32 read_ptr, write_ptr, read_offset, buf_size, unhandled_len, base_addr;
	u32 location, val_location;
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
		location = mmbi->bmc_int_location;
		val_location = MMBI_BMC_INT_VAL_OFFSET;
	} else {
		read_ptr = chan->buffer_desc.h_rws_p;
		write_ptr = chan->buffer_desc.h_ros_p;
		buf_size = chan->b2h_l;
		base_addr = chan->b2h_ba_offset;
		location = mmbi->host_int_location;
		val_location = MMBI_HOST_INT_VAL_OFFSET;
	}

	unhandled_len = mmbi_channel_unhandled_length(mmbi->desc_virt + read_ptr,
						      mmbi->desc_virt + write_ptr,
								      buf_size);

	if (unhandled_len == 0) {
		// dev_info(dev, "%s: no data to read for chan %u\n", __func__, chan->index);
		return 0;
	}

	desc_virt = mmbi->desc_virt + base_addr;
	data_len = 0;
	read_offset = MMBI_PTR_ADDR_ALIGN(be32_to_cpu(ioread32be(mmbi->desc_virt + read_ptr + sizeof(u32))));
	/* MMBI read/write has 4byte unit, so reading header will never have bundary issues */
	mmbi_parse_hdr(desc_virt + read_offset, &data_len, &padding, &protocol);

	if (data_len > read_len) {
		dev_warn(dev, "%s: not enough space(%ld) to read data(%u) for chan %u\n",
			 __func__, read_len, data_len, chan->index);
		dev_warn(dev, "%s: read_offset %d padding %d protocol %d buf_size %d", __func__, read_offset, padding, protocol, buf_size);
		return -EINVAL;
	}
	pkt_len = MMBI_PKT_HDR_SIZE + data_len + padding;

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
		dev_warn(dev, "%s: unsupported protocol %u on chan %u, discard packet\n",
			 __func__, protocol, chan->index);
	}
	mmbi_update_rd_ptr(mmbi->desc_virt + read_ptr, pkt_len, buf_size);
	mmbi_set_int_value(mmbi, val_location, location, BIT(chan->index));

	return protocol == MMBI_PROTOCOL_MCTP ? data_len : 0;
}

static int mmbi_write(struct mmbi_chan_desc *chan, const u8 *data, size_t data_len, u8 protocol)
{
	struct mmbi_ins_desc *mmbi = chan->mmbi;
	struct device *dev = chan->mmbi->dev;
	u32 read_ptr, write_ptr, write_offset, buf_size, avail_len, val, base_addr, total_len;
	u32 location, val_location;
	size_t first_part;
	u8 padding;
	u8 __iomem *desc_virt;

	if (data_len == 0)
		return -EINVAL;

	if (!chan->peer_ready) {
		dev_info(dev, "%s: chan %d peer not ready to receive data", __func__, chan->index);
		return -EBUSY;
	}

	if (mmbi->role == MMBI_ROLE_BMC) {
		read_ptr = chan->buffer_desc.h_rws_p;
		write_ptr = chan->buffer_desc.h_ros_p;
		buf_size = chan->b2h_l;
		base_addr = chan->b2h_ba_offset;
		location = mmbi->bmc_int_location;
		val_location = MMBI_BMC_INT_VAL_OFFSET;
	} else {
		read_ptr = chan->buffer_desc.h_ros_p;
		write_ptr = chan->buffer_desc.h_rws_p;
		buf_size = chan->h2b_l;
		base_addr = chan->h2b_ba_offset;
		location = mmbi->host_int_location;
		val_location = MMBI_HOST_INT_VAL_OFFSET;
	}

	avail_len = mmbi_channel_avail_length(mmbi->desc_virt + read_ptr,
					      mmbi->desc_virt + write_ptr,
					      buf_size);

	total_len = data_len + ((protocol == MMBI_PROTOCOL_MCTP) ? 4 : 0);

	/* at least 8bytes for 4byte header and 4byte data */
	if (avail_len < MMBI_PKT_MIN_SIZE) {
		dev_info(dev, "%s: no available buffer to write for chan %u\n",
			 __func__, chan->index);
		return -EAGAIN;
	}

	/* clear pending interrupt  */
	mmbi_clr_pending_int(mmbi, chan->index);
	if (total_len > avail_len) {
		dev_info(dev, "%s: no available buffer to write for chan %u, requested %d, avail %d\n",
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
		iowrite8((val >> 14) & 0xff, desc_virt + (write_offset % buf_size));
		write_offset++;
		iowrite8((val >> 6) & 0xff, desc_virt + (write_offset % buf_size));
		write_offset++;
		iowrite8(((val << 2) & 0xff) | padding, desc_virt + (write_offset % buf_size));
		write_offset++;
		iowrite8(MMBI_PKT_PROTOCOL_MASK & protocol, desc_virt + (write_offset % buf_size));
		write_offset++;
	}

	write_offset %= buf_size;
	if (write_offset + data_len <= buf_size) {
		memcpy_toio(desc_virt + write_offset, data, data_len);
	} else {
		first_part = buf_size - write_offset;
		memcpy_toio(desc_virt + write_offset, data, first_part);
		memcpy_toio(desc_virt, data + first_part, data_len - first_part);
	}

	mmbi_update_wr_ptr(mmbi->desc_virt + write_ptr, total_len, buf_size);
	mmbi_set_int_value(mmbi, val_location, location, BIT(chan->index));
	return data_len;
}

static int mmbi_misc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct mmbi_chan_desc *chan_desc = container_of(miscdev, struct mmbi_chan_desc, miscdev);

	file->private_data = chan_desc;
	mmbi_channel_state_update(chan_desc, chan_desc->mmbi->desc_virt);
	return 0;
}

static ssize_t mmbi_misc_read(struct file *file, char __user *buf, size_t count,
			      loff_t *ppos)
{
	struct mmbi_chan_desc *chan_desc = file->private_data;
	ssize_t ret, offset;
	unsigned long flags;
	void *kbuf;

	if (count == 0)
		return 0;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	offset = 0;
	spin_lock_irqsave(&chan_desc->rx_lock, flags);
	while (offset < count) {
		ret = mmbi_read(chan_desc, kbuf, count - offset);
		if (ret == 0)
			break;

		if (ret < 0) {
			offset = ret;
			break;
		}

		if (copy_to_user(buf + offset, kbuf, ret)) {
			dev_err(chan_desc->mmbi->dev, "copy to user failed\n");
			offset = -EFAULT;
			break;
		}

		offset += ret;
	}
	chan_desc->rx_ready = false;
	spin_unlock_irqrestore(&chan_desc->rx_lock, flags);
	// pr_info("%s: read %zd bytes for chan %u\n", __func__, offset, chan_desc->index);
	kfree(kbuf);
	return offset;
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
	ret = mmbi_write(chan_desc, kbuf, count, MMBI_PROTOCOL_MCTP);
	kfree(kbuf);
	return ret;
}

static __poll_t mmbi_misc_poll(struct file *file, struct poll_table_struct *pt)
{
	struct mmbi_chan_desc *chan_desc = file->private_data;
	struct mmbi_ins_desc *mmbi = chan_desc->mmbi;
	struct device *dev = mmbi->dev;
	__poll_t ret = 0;
	u8 __iomem *host_rws_virt, *host_ros_virt;

	mmbi_channel_state_update(chan_desc, mmbi->desc_virt);

	if (chan_desc->state != NORMAL_RUNTIME && chan_desc->state != RESET_REQ_BY_BMC) {
		dev_info(dev, "%s: chan %u invalid state 0x%x, not ready for polling\n",
			 __func__, chan_desc->index, chan_desc->state);
		return ret;
	}

	host_rws_virt = mmbi->desc_virt + chan_desc->buffer_desc.h_rws_p;
	host_ros_virt = mmbi->desc_virt + chan_desc->buffer_desc.h_ros_p;

	poll_wait(file, &chan_desc->rx_wait, pt);
	if (chan_desc->rx_ready)
		ret |= POLL_IN;

	return ret;
}

static const struct file_operations mmbi_fops = {
	.owner = THIS_MODULE,
	.open = mmbi_misc_open,
	.read = mmbi_misc_read,
	.write = mmbi_misc_write,
	.poll = mmbi_misc_poll,
	.release = NULL,
};

static int mmbi_instance_init_miscdev(struct mmbi_ins_desc *mmbi)
{
	int ret, i;
	struct mmbi_chan_desc *chan_desc;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		chan_desc = &mmbi->chan_desc[i];
		chan_desc->miscdev.name =
			devm_kasprintf(mmbi->dev, GFP_KERNEL, "mmbi%u-ch%u",
				       chan_desc->mmbi->ins_id, chan_desc->index);

		if (!chan_desc->miscdev.name) {
			dev_err(mmbi->dev, "%s: failed to allocate misc device name for chan %u\n",
				__func__, chan_desc->index);
			return -ENOMEM;
		}
		chan_desc->miscdev.minor = MISC_DYNAMIC_MINOR;
		chan_desc->miscdev.fops = &mmbi_fops;
		chan_desc->miscdev.parent = mmbi->dev;

		ret = misc_register(&chan_desc->miscdev);
		if (ret) {
			dev_err(mmbi->dev, "%s: failed to register misc device for chan %u\n",
				__func__, chan_desc->index);
		}
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
		prev_state = chan->state;
		buf_desc = &chan->buffer_desc;
		host_rws_vmem = desc_virt + buf_desc->h_rws_p;
		host_ros_vmem = desc_virt + buf_desc->h_ros_p;
		host_rws_val = ioread8(host_rws_vmem);
		host_ros_val = ioread8(host_ros_vmem);
		chan->state = (MMBI_STATE_GET_IF_UP(host_ros_val) << 2) |
			      (MMBI_STATE_GET_RST(host_ros_val) << 2) |
			      MMBI_STATE_GET_IF_UP(host_rws_val) |
			      MMBI_STATE_GET_RST(host_rws_val);
	} else {
		dev_err(chan->mmbi->dev, "\t%s: unsupported buffer type %d\n",
			__func__, chan->buffer_type);
		chan->state = POWER_UP_OR_ERROR;
	}
}
EXPORT_SYMBOL_GPL(mmbi_channel_state_update);

void mmbi_clr_pending_int(struct mmbi_ins_desc *mmbi, u8 idx)
{
	unsigned long flags;

	if (mmbi->mmbi_version == MMBI_VERSION_1_1) {
		spin_lock_irqsave(&mmbi->irq_lock, flags);
		mmbi->pending_int &= ~BIT(idx);
		spin_unlock_irqrestore(&mmbi->irq_lock, flags);
	}
}
EXPORT_SYMBOL_GPL(mmbi_clr_pending_int);

void mmbi_set_int_value(struct mmbi_ins_desc *mmbi, u32 val_location, u32 location, u8 val)
{
	unsigned long flags;
	u8 int_val;

	if (mmbi->mmbi_version == MMBI_VERSION_1_0) {
		/* trigger interrupt directly */
		if (mmbi->raise_interrupt)
			mmbi->raise_interrupt(mmbi->dev, mmbi->desc_virt, val_location, val);
	} else if (mmbi->mmbi_version == MMBI_VERSION_1_1) {
		spin_lock_irqsave(&mmbi->irq_lock, flags);

		/* previous channel interrupt has been completed */
		if (ioread8(mmbi->desc_virt + val_location) == 0) {
			/* update pending interrupts */
			int_val = val | mmbi->pending_int;
			if (int_val) {
				iowrite8(int_val, mmbi->desc_virt + val_location);
				mmbi->pending_int = 0;
				/* trigger interrupt */
				if (mmbi->raise_interrupt)
					mmbi->raise_interrupt(mmbi->dev, mmbi->desc_virt, location, val);
			}
		} else {
			/* accumulate pending interrupts until the previous one is handled */
			mmbi->pending_int |= val;
			schedule_delayed_work(&mmbi->irq_pending_work, msecs_to_jiffies(1));
		}

		spin_unlock_irqrestore(&mmbi->irq_lock, flags);
	}
}
EXPORT_SYMBOL_GPL(mmbi_set_int_value);

int mmbi_instance_init(struct mmbi_ins_desc *mmbi)
{
	int rc;

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

	spin_lock_init(&mmbi->irq_lock);
	INIT_WORK(&mmbi->work, mmbi_instance_irq);
	INIT_DELAYED_WORK(&mmbi->irq_pending_work, mmbi_instance_pending_irq);

	mmbi->ins_id = ida_alloc(&mmbi_ida, GFP_KERNEL);
	if (mmbi->ins_id < 0) {
		rc = mmbi->ins_id;
		goto out_fail;
	}

	mmbi->pending_int = 0;
	if (mmbi->role == MMBI_ROLE_BMC)
		rc = mmbi_instance_init_bmc(mmbi);
	else
		rc = mmbi_instance_init_host(mmbi);

	if (rc)
		goto ida_free;

	rc = mmbi_instance_init_miscdev(mmbi);
	if (rc)
		goto ida_free;

	return 0;

ida_free:
	ida_free(&mmbi_ida, mmbi->ins_id);
out_fail:
	dev_err(mmbi->dev, "%s: failed to init MMBI instance\n",
		__func__);
	return rc;
}
EXPORT_SYMBOL_GPL(mmbi_instance_init);
