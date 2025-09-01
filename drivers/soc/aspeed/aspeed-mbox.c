// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright Aspeed Technology Inc. (C) 2025. All rights reserved
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/sched/signal.h>

#define __ASPEED_MBOX_IOCTL_MAGIC	'X'
#define ASPEED_MBOX_IOCTL_CAPS		_IOR(__ASPEED_MBOX_IOCTL_MAGIC, 0, uint32_t[4])
#define ASPEED_MBOX_IOCTL_SEND		_IOW(__ASPEED_MBOX_IOCTL_MAGIC, 1, uint32_t[8])
#define ASPEED_MBOX_IOCTL_RECV		_IOR(__ASPEED_MBOX_IOCTL_MAGIC, 2, uint32_t[8])

/**
 * struct aspeed_mem - Description of a ASPEED memory buffer
 * @buf:	Shared memory base address
 * @size:	Shared memory byte size
 */
struct aspeed_mem {
	void __iomem *buf;
	phys_addr_t phys_addr;
	resource_size_t size;
};

/*
 * Message prototype of IPC. It should be defined per your usage.
 *  cmd: type of message
 *  len: length of message
 */
struct ast_mbox_msg {
	u32 cmd;
	u32 len;
};

/*
 * struct ast_mbox_chan - Description of a ASPEED mailbox channel
 * @cl:	Mailbox client
 * @mdev: Misc device
 * @chan: Mailbox channel
 * @tx_base: Information of tx shmem
 * @rx_base: Information of rx shmem
 * @rx_buffer: buffer to store data on callback
 * @rx_wait: Wait queue for receiving messages
 * @rx_msg_lock: Spinlock to protect rx_buffer
 */
struct ast_mbox_info {
	struct device		*dev;
	struct mbox_client	cl;
	struct miscdevice	mdev;
	struct mbox_chan	*chan;
	struct aspeed_mem	tx_base;
	struct aspeed_mem	rx_base;
	char			*rx_buffer;
	wait_queue_head_t	rx_wait;
	spinlock_t		rx_msg_lock;	/* spinlock to protect rx_buffer */
};

static ssize_t mbox_read(struct file *fp, char __user *buf, size_t nbytes, loff_t *off)
{
	struct ast_mbox_info *info = container_of(fp->private_data, struct ast_mbox_info, mdev);
	int ret;

	if (nbytes == 0)
		return 0;

	if (!info->rx_base.buf) {
		dev_err(info->dev, "No RX shmem\n");
		return -EINVAL;
	}

	if (nbytes > info->rx_base.size) {
		dev_warn(info->dev, "Read size %zu exceeds RX shmem size %llu\n",
			 nbytes, info->rx_base.size);
		nbytes = info->rx_base.size;
	}

	ret = copy_to_user((void __user *)buf, info->rx_base.buf, nbytes);
	if (ret)
		return -EFAULT;

	return nbytes;
}

static ssize_t mbox_write(struct file *fp, const char *buf, size_t nbytes, loff_t *off)
{
	struct ast_mbox_info *info = container_of(fp->private_data, struct ast_mbox_info, mdev);
	int ret;

	if (nbytes == 0)
		return 0;

	if (!info->tx_base.buf) {
		dev_err(info->dev, "No TX shmem\n");
		return -EINVAL;
	}

	if (nbytes > info->tx_base.size) {
		dev_warn(info->dev, "Write size %zu exceeds TX shmem size %llu\n",
			 nbytes, info->tx_base.size);
		nbytes = info->tx_base.size;
	}

	ret = copy_from_user(info->tx_base.buf, (void __user *)buf, nbytes);
	if (ret)
		return -EFAULT;

	return nbytes;
}

static __poll_t mbox_poll(struct file *fp, struct poll_table_struct *wait)
{
	struct ast_mbox_info *info = container_of(fp->private_data, struct ast_mbox_info, mdev);
	__poll_t mask = 0;

	poll_wait(fp, &info->rx_wait, wait);

	if (info->rx_buffer)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static long mbox_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct ast_mbox_info *info = container_of(fp->private_data, struct ast_mbox_info, mdev);
	u32 *data;
	u32 msg[8];
	unsigned long flags;
	int ret = 0;

	switch (cmd) {
	case ASPEED_MBOX_IOCTL_CAPS:
		data = (u32 *)arg;
		data[0] = (info->tx_base.phys_addr) >> 8;
		data[1] = info->tx_base.size;
		data[2] = (info->rx_base.phys_addr) >> 8;
		data[3] = info->rx_base.size;
		break;
	case ASPEED_MBOX_IOCTL_SEND:
		ret = copy_from_user(msg, (void __user *)arg, sizeof(msg));
		if (ret) {
			dev_dbg(info->dev, "Failed to copy message from user\n");
			return -EFAULT;
		}

		ret = mbox_send_message(info->chan, msg);
		if (ret < 0)
			dev_dbg(info->dev, "Failed to send message via mailbox\n");
		break;
	case ASPEED_MBOX_IOCTL_RECV:
		spin_lock_irqsave(&info->rx_msg_lock, flags);
		if (!info->rx_buffer) {
			spin_unlock_irqrestore(&info->rx_msg_lock, flags);
			dev_dbg(info->dev, "No message received\n");
			return -EAGAIN;
		}
		ret = copy_to_user((void __user *)arg, info->rx_buffer, sizeof(msg));
		info->rx_buffer = NULL;
		spin_unlock_irqrestore(&info->rx_msg_lock, flags);
		if (ret) {
			dev_dbg(info->dev, "Failed to copy message to user\n");
			return -EFAULT;
		}
		break;
	default:
		dev_err(info->dev, "Unsupported cmd: %x\n", cmd);
		ret = -EINVAL;
	}

	return ret;
}

static const struct file_operations aspeed_mbox_fops = {
	.owner		= THIS_MODULE,
	.read		= mbox_read,
	.write		= mbox_write,
	.poll		= mbox_poll,
	.unlocked_ioctl	= mbox_ioctl,
};

static void mbox_rx_callback(struct mbox_client *client, void *message)
{
	struct ast_mbox_info *info = container_of(client, struct ast_mbox_info, cl);
	unsigned long flags;

	spin_lock_irqsave(&info->rx_msg_lock, flags);
	info->rx_buffer = message;
	spin_unlock_irqrestore(&info->rx_msg_lock, flags);

	wake_up_interruptible(&info->rx_wait);
}

static int aspeed_mbox_probe(struct platform_device *pdev)
{
	struct ast_mbox_info *info;
	struct resource *res;
	struct device *dev = &pdev->dev;
	int ret;
	u32 tx_tout = -1;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->tx_base.buf = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (PTR_ERR(info->tx_base.buf) == -EBUSY) {
		/* if reserved area in SRAM, try just ioremap */
		info->tx_base.size = resource_size(res);
		info->tx_base.buf = devm_ioremap(dev, res->start, info->tx_base.size);
	}
	if (IS_ERR(info->tx_base.buf)) {
		info->tx_base.buf = NULL;
	} else {
		info->tx_base.phys_addr = res->start;
		info->tx_base.size = resource_size(res);
	}

	info->rx_base.buf = devm_platform_get_and_ioremap_resource(pdev, 1, &res);
	if (PTR_ERR(info->rx_base.buf) == -EBUSY) {
		/* if reserved area in SRAM, try just ioremap */
		info->rx_base.size = resource_size(res);
		info->rx_base.buf = devm_ioremap(dev, res->start, info->rx_base.size);
	}
	if (IS_ERR(info->rx_base.buf)) {
		info->rx_base = info->tx_base;
	} else {
		info->rx_base.phys_addr = res->start;
		info->rx_base.size = resource_size(res);
	}

	if (device_property_read_u32(dev, "aspeed,tx-timeout", &tx_tout))
		tx_tout = -1;

	dev_info(dev, "TX shmem: phys 0x%pa size %llu\n",
		 &info->tx_base.phys_addr, info->tx_base.size);
	dev_info(dev, "RX shmem: phys 0x%pa size %llu\n",
		 &info->rx_base.phys_addr, info->rx_base.size);
	dev_info(dev, "TX timeout: %u ms\n", tx_tout);

	info->cl.dev		= dev;
	info->cl.rx_callback	= mbox_rx_callback;
	info->cl.tx_block	= true;
	info->cl.knows_txdone	= false;
	info->cl.tx_tout	= tx_tout;

	info->chan = mbox_request_channel(&info->cl, 0);
	if (IS_ERR(info->chan)) {
		dev_err(dev, "failed to request channel err %ld\n",
			PTR_ERR(info->chan));
		return -EPROBE_DEFER;
	}

	info->rx_buffer = NULL;
	init_waitqueue_head(&info->rx_wait);
	spin_lock_init(&info->rx_msg_lock);
	info->dev = dev;
	platform_set_drvdata(pdev, info);

	info->mdev.parent = dev;
	info->mdev.minor = MISC_DYNAMIC_MINOR;
	info->mdev.name = dev_name(dev);
	info->mdev.fops = &aspeed_mbox_fops;
	ret = misc_register(&info->mdev);
	if (ret) {
		dev_err(dev, "failed to register misc device\n");
		return ret;
	}

	return 0;
}

static int aspeed_mbox_remove(struct platform_device *pdev)
{
	struct ast_mbox_info *info = platform_get_drvdata(pdev);

	mbox_free_channel(info->chan);
	misc_deregister(&info->mdev);

	return 0;
}

static const struct of_device_id mbox_cl_match[] = {
	{ .compatible = "aspeed,aspeed-mbox" },
	{},
};
MODULE_DEVICE_TABLE(of, mbox_test_match);

static struct platform_driver aspeed_mbox_driver = {
	.driver = {
		.name = "aspeed_mbox_client",
		.of_match_table = mbox_cl_match,
	},
	.probe	= aspeed_mbox_probe,
	.remove	= aspeed_mbox_remove,
};
module_platform_driver(aspeed_mbox_driver);

MODULE_AUTHOR("Jammy Huang <jammy_huang@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED MBOX CLIENT driver");
MODULE_LICENSE("GPL");
