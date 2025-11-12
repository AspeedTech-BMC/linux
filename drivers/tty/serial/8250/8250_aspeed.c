// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/dma-mapping.h>
#include <linux/tty_flip.h>
#include <linux/pm_runtime.h>
#include <linux/soc/aspeed/aspeed-udma.h>

#include "8250.h"

#define DEVICE_NAME "aspeed-uart"
#define UNKNOWN 0
#define AST2500_PLAT 1
#define AST2600_PLAT 2
#define AST2700_PLAT 3

/* offsets for the aspeed virtual uart registers */
#define VUART_GCRA	0x20
#define   VUART_GCRA_VUART_EN			BIT(0)
#define   VUART_GCRA_SIRQ_POLARITY		BIT(1)
#define   VUART_GCRA_CHARACTER_TIMEOUT_TIME_MASK	GENMASK(3, 2)
#define   VUART_GCRA_DISABLE_HOST_TX_DISCARD	BIT(5)
#define VUART_GCRB	0x24
#define   VUART_GCRB_HOST_SIRQ_MASK		GENMASK(7, 4)
#define   VUART_GCRB_HOST_SIRQ_SHIFT		4
#define VUART_ADDRL	0x28
#define VUART_ADDRH	0x2c
#define VUART_GCRG	0x38
#define   VUART_GCRG_CHARACTER_TIMEOUT_TIME_CONTROL	BIT(1)

#define DMA_TX_BUFSZ	PAGE_SIZE
#define DMA_RX_BUFSZ	(64 * 1024)

struct uart_ops ast8250_pops;

struct ast8250_vuart {
	u32 port;
	u32 sirq;
	u32 sirq_pol;
	bool character_timeout_time_en;
};

struct ast8250_udma {
	u32 ch;

	u32 tx_fifosz;
	u32 rx_fifosz;

	dma_addr_t tx_addr;
	dma_addr_t rx_addr;

	struct kfifo *tx_fifo;
	struct kfifo *rx_fifo;

	bool tx_tmout_dis;
	bool rx_tmout_dis;
};

struct ast8250_data {
	int line;

	u8 __iomem *regs;

	bool is_vuart;
	bool use_dma;

	struct reset_control *rst;
	struct clk *clk;

	struct ast8250_vuart vuart;
	struct ast8250_udma dma;
};

static void ast8250_dma_tx_complete(int tx_fifo_rptr, void *id)
{
    unsigned long flags;
	struct uart_port *port = (struct uart_port*)id;
	struct ast8250_data *data = port->private_data;
	struct kfifo *tx_fifo = data->dma.tx_fifo;
	unsigned int len;

    spin_lock_irqsave(&port->lock, flags);

	len = kfifo_out(tx_fifo, NULL, tx_fifo_rptr);
	port->icount.tx += len;

	if (kfifo_len(tx_fifo) < WAKEUP_CHARS)
		uart_write_wakeup(port);

    spin_unlock_irqrestore(&port->lock, flags);
}

static void ast8250_dma_rx_complete(int rx_fifo_wptr, void *id)
{
	unsigned long flags;
	struct uart_port *up = (struct uart_port*)id;
	struct tty_port *tp = &up->state->port;
	struct ast8250_data *data = up->private_data;
	struct ast8250_udma *dma = &data->dma;
	struct kfifo *rx_fifo = dma->rx_fifo;
	u32 len = 0;
	u8 buf[128];

	spin_lock_irqsave(&up->lock, flags);

	dma_sync_single_for_cpu(up->dev,
			dma->rx_addr, dma->rx_fifosz, DMA_FROM_DEVICE);

	while (!kfifo_is_empty(rx_fifo)) {
		len = kfifo_out(rx_fifo, buf, sizeof(buf));
		tty_insert_flip_string(tp, buf, len);
		up->icount.rx += len;
	}

	tty_flip_buffer_push(tp);

	spin_unlock_irqrestore(&up->lock, flags);
}

static void ast8250_dma_start_tx(struct uart_port *port)
{
	struct ast8250_data *data = port->private_data;
	struct ast8250_udma *dma = &data->dma;
	struct kfifo *tx_fifo = dma->tx_fifo;

	dma_sync_single_for_device(port->dev,
				   dma->tx_addr, dma->tx_fifosz, DMA_TO_DEVICE);

	aspeed_udma_set_tx_wptr(dma->ch, kfifo_len(tx_fifo));
}

static void ast8250_dma_pops_hook(struct uart_port *port)
{
	static int first = 1;

	if (first) {
		ast8250_pops = *port->ops;
		ast8250_pops.start_tx = ast8250_dma_start_tx;
	}

	first = 0;
	port->ops = &ast8250_pops;
}

static void ast8250_vuart_init(struct ast8250_data *data)
{
	u8 reg;
	struct ast8250_vuart *vuart = &data->vuart;

	/* IO port address */
	writeb((u8)(vuart->port >> 0), data->regs + VUART_ADDRL);
	writeb((u8)(vuart->port >> 8), data->regs + VUART_ADDRH);

	/* SIRQ number */
	reg = readb(data->regs + VUART_GCRB);
	reg &= ~VUART_GCRB_HOST_SIRQ_MASK;
	reg |= ((vuart->sirq << VUART_GCRB_HOST_SIRQ_SHIFT) & VUART_GCRB_HOST_SIRQ_MASK);
	writeb(reg, data->regs + VUART_GCRB);

	/* SIRQ polarity */
	reg = readb(data->regs + VUART_GCRA);
	if (vuart->sirq_pol)
		reg |= VUART_GCRA_SIRQ_POLARITY;
	else
		reg &= ~VUART_GCRA_SIRQ_POLARITY;
	writeb(reg, data->regs + VUART_GCRA);

	if (vuart->character_timeout_time_en) {
		/* Character timeout time */
		reg = readb(data->regs + VUART_GCRA);
		reg |= VUART_GCRA_CHARACTER_TIMEOUT_TIME_MASK;
		writeb(reg, data->regs + VUART_GCRA);

		/* Character timeout time by LCLK control bit */
		reg = readb(data->regs + VUART_GCRG);
		reg |= VUART_GCRG_CHARACTER_TIMEOUT_TIME_CONTROL;
		writeb(reg, data->regs + VUART_GCRG);
	}
}

static void ast8250_vuart_set_host_tx_discard(struct ast8250_data *data, bool discard)
{
	u8 reg;

	reg = readb(data->regs + VUART_GCRA);
	if (discard)
		reg &= ~VUART_GCRA_DISABLE_HOST_TX_DISCARD;
	else
		reg |= VUART_GCRA_DISABLE_HOST_TX_DISCARD;
	writeb(reg, data->regs + VUART_GCRA);
}

static void ast8250_vuart_set_enable(struct ast8250_data *data, bool enable)
{
	u8 reg;

	reg = readb(data->regs + VUART_GCRA);
	if (enable)
		reg |= VUART_GCRA_VUART_EN;
	else
		reg &= ~VUART_GCRA_VUART_EN;
	writeb(reg, data->regs + VUART_GCRA);
}

static int ast8250_handle_irq(struct uart_port *port)
{
	u32 iir = port->serial_in(port, UART_IIR);
	return serial8250_handle_irq(port, iir);
}

static int ast8250_startup(struct uart_port *port)
{
	int rc = 0;
	struct ast8250_data *data = port->private_data;
	struct ast8250_udma *dma;

	if (data->is_vuart)
		ast8250_vuart_set_host_tx_discard(data, false);

	if (data->use_dma) {
		dma = &data->dma;

		dma->tx_fifosz = DMA_TX_BUFSZ;
		dma->rx_fifosz = DMA_RX_BUFSZ;

		if (kfifo_alloc(dma->tx_fifo, dma->tx_fifosz, GFP_KERNEL)) {
			dev_err(port->dev, "failed to allocate TX DMA ring buffer\n");
			rc = -ENOMEM;
			goto out;
		}

		if (kfifo_alloc(dma->rx_fifo, dma->rx_fifosz, GFP_KERNEL)) {
			dev_err(port->dev, "failed to allocate RX DMA ring buffer\n");
			rc = -ENOMEM;
			goto free_tx_fifo;
		}

		dma->tx_addr = dma_map_single(port->dev, dma->tx_fifo->kfifo.data,
					      dma->tx_fifosz, DMA_TO_DEVICE);
		if (dma_mapping_error(port->dev, dma->tx_addr)) {
			dev_err(port->dev, "failed to map streaming TX DMA region\n");
			rc = -ENOMEM;
			goto free_rx_fifo;
		}

		dma->rx_addr = dma_map_single(port->dev, dma->rx_fifo->kfifo.data,
					      dma->rx_fifosz, DMA_FROM_DEVICE);
		if (dma_mapping_error(port->dev, dma->rx_addr)) {
			dev_err(port->dev, "failed to map streaming RX DMA region\n");
			rc = -ENOMEM;
			goto free_rx_fifo;
		}

		rc = aspeed_udma_request_tx_chan(dma->ch, dma->tx_addr,
				dma->tx_fifo, dma->tx_fifosz, ast8250_dma_tx_complete, port, dma->tx_tmout_dis);
		if (rc) {
			dev_err(port->dev, "failed to request DMA TX channel\n");
			goto free_rx_fifo;
		}

		rc = aspeed_udma_request_rx_chan(dma->ch, dma->rx_addr,
				dma->rx_fifo, dma->rx_fifosz, ast8250_dma_rx_complete, port, dma->rx_tmout_dis);
		if (rc) {
			dev_err(port->dev, "failed to request DMA RX channel\n");
			goto free_rx_fifo;
		}

		ast8250_dma_pops_hook(port);

		aspeed_udma_tx_chan_ctrl(dma->ch, ASPEED_UDMA_OP_ENABLE);
		aspeed_udma_rx_chan_ctrl(dma->ch, ASPEED_UDMA_OP_ENABLE);
	}

	memset(&port->icount, 0, sizeof(port->icount));
	return serial8250_do_startup(port);

free_rx_fifo:
	kfifo_free(dma->rx_fifo);

free_tx_fifo:
	kfifo_free(dma->tx_fifo);

out:
	return rc;
}

static void ast8250_shutdown(struct uart_port *port)
{
	int rc;
	struct ast8250_data *data = port->private_data;
	struct ast8250_udma *dma;

	if (data->use_dma) {
		dma = &data->dma;

		aspeed_udma_tx_chan_ctrl(dma->ch, ASPEED_UDMA_OP_RESET);
		aspeed_udma_rx_chan_ctrl(dma->ch, ASPEED_UDMA_OP_RESET);

		aspeed_udma_tx_chan_ctrl(dma->ch, ASPEED_UDMA_OP_DISABLE);
		aspeed_udma_rx_chan_ctrl(dma->ch, ASPEED_UDMA_OP_DISABLE);

		rc = aspeed_udma_free_tx_chan(dma->ch);
		if (rc)
			dev_err(port->dev, "failed to free DMA TX channel, rc=%d\n", rc);

		rc = aspeed_udma_free_rx_chan(dma->ch);
		if (rc)
			dev_err(port->dev, "failed to free DMA TX channel, rc=%d\n", rc);

		dma_unmap_single(port->dev, dma->tx_addr,
				dma->tx_fifosz, DMA_TO_DEVICE);
		dma_unmap_single(port->dev, dma->rx_addr,
				dma->rx_fifosz, DMA_FROM_DEVICE);

		kfree(dma->tx_fifo);
		kfree(dma->rx_fifo);
	}

	if (data->is_vuart)
		ast8250_vuart_set_host_tx_discard(data, true);

	serial8250_do_shutdown(port);
}

static int __maybe_unused ast8250_suspend(struct device *dev)
{
	struct ast8250_data *data = dev_get_drvdata(dev);
	serial8250_suspend_port(data->line);
	return 0;
}

static int __maybe_unused ast8250_resume(struct device *dev)
{
	struct ast8250_data *data = dev_get_drvdata(dev);
	serial8250_resume_port(data->line);
	return 0;
}

static int ast8250_probe(struct platform_device *pdev)
{
	int rc;
	struct uart_8250_port uart = {};
	struct uart_port *port = &uart.port;
	struct device *dev = &pdev->dev;
	struct ast8250_data *data;
	uint32_t plat = (unsigned long)of_device_get_match_data(dev);

	struct resource *res;
	u32 irq;

	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (rc) {
		dev_err(dev, "cannot set 64-bits DMA mask\n");
		return rc;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
	    return -ENOMEM;

	data->dma.rx_fifo = devm_kzalloc(dev, sizeof(data->dma.rx_fifo), GFP_KERNEL);
	if (!data->dma.rx_fifo)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		if (irq != -EPROBE_DEFER)
			dev_err(dev, "failed to get IRQ number\n");
		return irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "failed to get register base\n");
		return -ENODEV;
	}

	data->regs = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(data->regs)) {
		dev_err(dev, "failed to map registers\n");
		return PTR_ERR(data->regs);
	}

	data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(data->clk)) {
		dev_err(dev, "failed to get clocks\n");
		return -ENODEV;
	}

	rc = clk_prepare_enable(data->clk);
	if (rc) {
		dev_err(dev, "failed to enable clock\n");
		return rc;
	}

	data->rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (!IS_ERR(data->rst))
		reset_control_deassert(data->rst);

	data->is_vuart = of_property_read_bool(dev->of_node, "virtual");
	if (data->is_vuart) {
		rc = of_property_read_u32(dev->of_node, "port", &data->vuart.port);
		if (rc) {
			dev_err(dev, "failed to get VUART port address\n");
			return -ENODEV;
		}

		rc = of_property_read_u32(dev->of_node, "sirq", &data->vuart.sirq);
		if (rc) {
			dev_err(dev, "failed to get VUART SIRQ number\n");
			return -ENODEV;
		}

		rc = of_property_read_u32(dev->of_node, "sirq-polarity", &data->vuart.sirq_pol);
		if (rc) {
			dev_err(dev, "failed to get VUART SIRQ polarity\n");
			return -ENODEV;
		}

		if (plat == AST2700_PLAT)
			data->vuart.character_timeout_time_en = true;
		else
			data->vuart.character_timeout_time_en = false;

		ast8250_vuart_init(data);
		ast8250_vuart_set_host_tx_discard(data, true);
		ast8250_vuart_set_enable(data, true);
	}

	data->use_dma = of_property_read_bool(dev->of_node, "dma-mode");
	if (data->use_dma) {
		dev_warn(dev, "DMA mode not ready\n");
		data->use_dma = false;
		/*
		rc = of_property_read_u32(dev->of_node, "dma-channel", &data->dma.ch);
		if (rc) {
			dev_err(dev, "failed to get DMA channel\n");
			return -ENODEV;
		}

		data->dma.tx_tmout_dis = of_property_read_bool(dev->of_node, "dma-tx-timeout-disable");
		data->dma.rx_tmout_dis = of_property_read_bool(dev->of_node, "dma-rx-timeout-disable");
		*/
	}

	spin_lock_init(&port->lock);
	port->dev = dev;
	port->type = PORT_16550A;
	port->irq = irq;
	port->line = of_alias_get_id(dev->of_node, "serial");
	port->handle_irq = ast8250_handle_irq;
	port->mapbase = res->start;
	port->mapsize = resource_size(res);
	port->membase = data->regs;
	port->uartclk = clk_get_rate(data->clk);
	port->regshift = 2;
	port->iotype = UPIO_MEM32;
	port->flags = UPF_FIXED_TYPE | UPF_FIXED_PORT | UPF_SHARE_IRQ;
	port->startup = ast8250_startup;
	port->shutdown = ast8250_shutdown;
	port->private_data = data;
	uart.bugs |= UART_BUG_TXRACE;

	data->line = serial8250_register_8250_port(&uart);
	if (data->line < 0) {
		dev_err(dev, "failed to register 8250 port\n");
		return data->line;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	platform_set_drvdata(pdev, data);
	return 0;
}

static void ast8250_remove(struct platform_device *pdev)
{
    struct ast8250_data *data = platform_get_drvdata(pdev);

	if (data->is_vuart)
		ast8250_vuart_set_enable(data, false);

    serial8250_unregister_port(data->line);
}

static const struct dev_pm_ops ast8250_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ast8250_suspend, ast8250_resume)
};

static const struct of_device_id ast8250_of_match[] = {
	{ .compatible = "aspeed,ast2500-uart", .data = (void *)AST2500_PLAT},
	{ .compatible = "aspeed,ast2600-uart", .data = (void *)AST2600_PLAT},
	{ .compatible = "aspeed,ast2700-uart", .data = (void *)AST2700_PLAT},
	{ },
};

static struct platform_driver ast8250_platform_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.pm = &ast8250_pm_ops,
		.of_match_table = ast8250_of_match,
	},
	.probe = ast8250_probe,
	.remove = ast8250_remove,
};

module_platform_driver(ast8250_platform_driver);

MODULE_AUTHOR("Chia-Wei Wang <chiawei_wang@aspeedtech.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Aspeed UART Driver");
