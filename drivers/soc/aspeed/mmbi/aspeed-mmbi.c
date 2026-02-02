// SPDX-License-Identifier: GPL-2.0+
/*
 * ASPEED MMBI (Memory Mapped Bus Interface) Protocol Driver
 *
 * Copyright (c) 2026 ASPEED Technology Inc.
 */

#include "aspeed-mmbi-host.h"
#include "linux/dev_printk.h"
#include <linux/device.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "aspeed-mmbi.h"
#include "aspeed-mmbi-bmc.h"

static DEFINE_IDA(mmbi_ida);

static void mmbi_instance_irq(struct work_struct *work)
{
	struct mmbi_ins_desc *mmbi = container_of(work, struct mmbi_ins_desc, work);

	dev_info(mmbi->dev, "%s: MMBI instance %u received interrupt\n", __func__, mmbi->ins_id);
	/* Handle MMBI interrupt here */
	if (mmbi->role == MMBI_ROLE_BMC)
		mmbi_instance_irq_bmc(mmbi);
	else
		mmbi_instance_irq_host(mmbi);
}

static int mmbi_instance_init_miscdev(struct mmbi_ins_desc *mmbi)
{
	int ret, i;
	char dev_name[20];
	struct mmbi_chan_desc *chan_desc;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		chan_desc = &mmbi->chan_desc[i];
		snprintf(dev_name, sizeof(dev_name), "mmbi_ins_%u_chan_%u",
			 chan_desc->mmbi->ins_id, chan_desc->index);

		chan_desc->miscdev.minor = MISC_DYNAMIC_MINOR;
		chan_desc->miscdev.name = dev_name;
		chan_desc->miscdev.fops = NULL; /* TODO: implement file operations if needed */
		chan_desc->miscdev.parent = mmbi->dev;

		ret = misc_register(&chan_desc->miscdev);
		if (ret) {
			dev_err(mmbi->dev, "%s: failed to register misc device for channel %u\n",
				__func__, chan_desc->index);
		}

		dev_info(mmbi->dev,
			 "%s: registered misc device %s for channel %u\n",
			 __func__, dev_name, chan_desc->index);
	}
	return 0;
}

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
		dev_info(chan->mmbi->dev,
			 "%s:   Channel %u state 0x%x -> 0x%x\n", __func__,
			 chan->index, prev_state, chan->state);
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

	pr_info("%s: MMBI instance %u set interrupt value 0x%x at location 0x%x\n",
		__func__, mmbi->ins_id, val, location);

	if (mmbi->mmbi_version == MMBI_VERSION_1_0) {
		/* trigger interrupt directly */
		if (mmbi->raise_interrupt)
			mmbi->raise_interrupt(mmbi->dev, mmbi->desc_virt, val_location, val);
	} else if (mmbi->mmbi_version == MMBI_VERSION_1_1) {
		spin_lock_irqsave(&mmbi->irq_lock, flags);

		/* previous channel interrupt has been completed */
		if (ioread8(mmbi->desc_virt + location) == 0) {
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
