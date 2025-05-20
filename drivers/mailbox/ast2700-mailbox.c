// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright Aspeed Technology Inc. (C) 2025. All rights reserved
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Each bit in the register represents an IPC ID */
#define IPCR_TX_TRIG		0x00
#define IPCR_TX_ENABLE		0x04
#define IPCR_RX_ENABLE		0x104
#define IPCR_TX_STATUS		0x08
#define IPCR_RX_STATUS		0x108
#define  RX_IRQ(n)		BIT(0 + 1 * (n))
#define  RX_IRQ_MASK		0xf
#define IPCR_TX_DATA		0x10
#define IPCR_RX_DATA		0x110

struct ast2700_mbox_data {
	u8 num_chans;
	u8 msg_size;
};

struct ast2700_mbox {
	struct mbox_controller mbox;
	const struct ast2700_mbox_data *drv_data;
	void __iomem *regs;
	u32 *rx_buff;
};

static inline int ch_num(struct mbox_chan *chan)
{
	return chan - chan->mbox->chans;
}

static inline int ast2700_mbox_tx_done(struct ast2700_mbox *mb, int idx)
{
	return !(readl(mb->regs + IPCR_TX_STATUS) & BIT(idx));
}

static irqreturn_t ast2700_mbox_irq(int irq, void *p)
{
	struct ast2700_mbox *mb = p;
	void __iomem *data_reg;
	int num_words;
	u32 *word_data;
	u32 status;
	int n;

	/* Only examine channels that are currently enabled. */
	status = readl(mb->regs + IPCR_RX_ENABLE) &
		 readl(mb->regs + IPCR_RX_STATUS);

	if (!(status & RX_IRQ_MASK))
		return IRQ_NONE;

	for (n = 0; n < mb->mbox.num_chans; ++n) {
		struct mbox_chan *chan = &mb->mbox.chans[n];

		if (!(status & RX_IRQ(n)))
			continue;

		for (data_reg = mb->regs + IPCR_RX_DATA + mb->drv_data->msg_size * n,
		     word_data = chan->con_priv,
		     num_words = (mb->drv_data->msg_size / sizeof(u32));
		     num_words; num_words--, data_reg += sizeof(u32), word_data++)
			*word_data = readl(data_reg);

		mbox_chan_received_data(chan, chan->con_priv);

		/* The IRQ can be cleared only once the FIFO is empty. */
		writel(RX_IRQ(n), mb->regs + IPCR_RX_STATUS);
	}

	return IRQ_HANDLED;
}

static int ast2700_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct ast2700_mbox *mb = dev_get_drvdata(chan->mbox->dev);
	void __iomem *data_reg;
	u32 *word_data;
	int num_words;
	int idx = ch_num(chan);

	if (!(readl(mb->regs + IPCR_TX_ENABLE) & BIT(idx))) {
		dev_warn(mb->mbox.dev, "%s: Ch-%d not enabled yet\n", __func__, idx);
		return -EBUSY;
	}

	if (!(ast2700_mbox_tx_done(mb, idx))) {
		dev_warn(mb->mbox.dev, "%s: Ch-%d last data has not finished\n", __func__, idx);
		return -EBUSY;
	}

	for (data_reg = mb->regs + IPCR_TX_DATA + mb->drv_data->msg_size * idx,
	     num_words = (mb->drv_data->msg_size / sizeof(u32)),
	     word_data = (u32 *)data;
	     num_words; num_words--, data_reg += sizeof(u32), word_data++)
		writel(*word_data, data_reg);

	writel(BIT(idx), mb->regs + IPCR_TX_TRIG);
	dev_dbg(mb->mbox.dev, "%s: Ch-%d sent\n", __func__, idx);

	return 0;
}

static int ast2700_mbox_startup(struct mbox_chan *chan)
{
	struct ast2700_mbox *mb = dev_get_drvdata(chan->mbox->dev);
	int idx = ch_num(chan);
	void __iomem *reg = mb->regs + IPCR_RX_ENABLE;

	writel_relaxed(readl_relaxed(reg) | BIT(idx), reg);

	return 0;
}

static void ast2700_mbox_shutdown(struct mbox_chan *chan)
{
	struct ast2700_mbox *mb = dev_get_drvdata(chan->mbox->dev);
	int idx = ch_num(chan);
	void __iomem *reg = mb->regs + IPCR_RX_ENABLE;

	writel_relaxed(readl_relaxed(reg) & ~BIT(idx), reg);
}

static bool ast2700_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct ast2700_mbox *mb = dev_get_drvdata(chan->mbox->dev);
	int idx = ch_num(chan);

	return ast2700_mbox_tx_done(mb, idx) ? true : false;
}

static const struct mbox_chan_ops ast2700_mbox_chan_ops = {
	.send_data	= ast2700_mbox_send_data,
	.startup	= ast2700_mbox_startup,
	.shutdown	= ast2700_mbox_shutdown,
	.last_tx_done	= ast2700_mbox_last_tx_done,
};

static int ast2700_mbox_probe(struct platform_device *pdev)
{
	struct ast2700_mbox *mb;
	const struct ast2700_mbox_data *drv_data;
	struct device *dev = &pdev->dev;
	int irq, ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	drv_data = (const struct ast2700_mbox_data *)device_get_match_data(&pdev->dev);

	mb = devm_kzalloc(dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;

	mb->mbox.chans = devm_kcalloc(&pdev->dev, drv_data->num_chans,
				      sizeof(*mb->mbox.chans), GFP_KERNEL);
	if (!mb->mbox.chans)
		return -ENOMEM;

	for (int i = 0; i < drv_data->num_chans; i++) {
		mb->mbox.chans[i].con_priv = devm_kcalloc(dev, drv_data->msg_size,
							  sizeof(u8), GFP_KERNEL);
		if (!mb->mbox.chans[i].con_priv)
			return -ENOMEM;
	}

	platform_set_drvdata(pdev, mb);

	mb->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mb->regs))
		return PTR_ERR(mb->regs);

	mb->drv_data = drv_data;
	mb->mbox.dev = dev;
	mb->mbox.num_chans = drv_data->num_chans;
	mb->mbox.ops = &ast2700_mbox_chan_ops;
	mb->mbox.txdone_irq = false;
	mb->mbox.txdone_poll = true;
	mb->mbox.txpoll_period = 5;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, ast2700_mbox_irq, 0, dev_name(dev), mb);
	if (ret)
		return ret;

	return devm_mbox_controller_register(dev, &mb->mbox);
}

static const struct ast2700_mbox_data ast2700_drv_data = {
	.num_chans = 4,
	.msg_size = 0x20,
};

static const struct of_device_id ast2700_mbox_of_match[] = {
	{ .compatible = "aspeed,ast2700-mailbox", .data = &ast2700_drv_data },
	{}
};
MODULE_DEVICE_TABLE(of, ast2700_mbox_of_match);

static struct platform_driver ast2700_mbox_driver = {
	.driver = {
		.name = "ast2700-mailbox",
		.of_match_table = ast2700_mbox_of_match,
	},
	.probe = ast2700_mbox_probe,
};
module_platform_driver(ast2700_mbox_driver);

MODULE_AUTHOR("Jammy Huang <jammy_huang@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED AST2700 IPC driver");
MODULE_LICENSE("GPL");
