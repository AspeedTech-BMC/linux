// SPDX-License-Identifier: GPL-2.0+
/*
 * Unified Aspeed eSPI driver (AST2500/AST2600/AST2700)
 *
 * This wraps the existing SoC-specific implementations behind a
 * single driver name and compatible strings. For now we keep the
 * full implementations in their original files and dispatch through
 * small per-SoC glue ops.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/reset.h>

#include "aspeed-espi.h"
#include "ast2500-espi.h"
#include "ast2600-espi.h"
#include "ast2700-espi.h"

struct aspeed_espi_ops {
	enum aspeed_espi_platform_id platform_id;
	void (*espi_pre_init)(struct aspeed_espi *espi);
	void (*espi_post_init)(struct aspeed_espi *espi);
	void (*espi_deinit)(struct aspeed_espi *espi);
	int (*espi_perif_probe)(struct aspeed_espi *espi);
	int (*espi_perif_remove)(struct aspeed_espi *espi);
	int (*espi_vw_probe)(struct aspeed_espi *espi);
	int (*espi_vw_remove)(struct aspeed_espi *espi);
	int (*espi_oob_probe)(struct aspeed_espi *espi);
	int (*espi_oob_remove)(struct aspeed_espi *espi);
	int (*espi_flash_probe)(struct aspeed_espi *espi);
	int (*espi_flash_remove)(struct aspeed_espi *espi);
	irqreturn_t (*espi_isr)(int irq, void *espi);
};

static const struct aspeed_espi_ops aspeed_espi_ast2500_ops = {
	.platform_id = ASPEED_ESPI_PLATFORM_ID_AST2500,
	.espi_pre_init = ast2500_espi_pre_init,
	.espi_post_init = ast2500_espi_post_init,
	.espi_deinit = ast2500_espi_deinit,
	.espi_perif_probe = ast2500_espi_perif_probe,
	.espi_perif_remove = ast2500_espi_perif_remove,
	.espi_vw_probe = ast2500_espi_vw_probe,
	.espi_vw_remove = ast2500_espi_vw_remove,
	.espi_oob_probe = ast2500_espi_oob_probe,
	.espi_oob_remove = ast2500_espi_oob_remove,
	.espi_flash_probe = ast2500_espi_flash_probe,
	.espi_flash_remove = ast2500_espi_flash_remove,
	.espi_isr = ast2500_espi_isr,
};

static const struct aspeed_espi_ops aspeed_espi_ast2600_ops = {
	.platform_id = ASPEED_ESPI_PLATFORM_ID_AST2600,
	.espi_pre_init = ast2600_espi_pre_init,
	.espi_post_init = ast2600_espi_post_init,
	.espi_deinit = ast2600_espi_deinit,
	.espi_perif_probe = ast2600_espi_perif_probe,
	.espi_perif_remove = ast2600_espi_perif_remove,
	.espi_vw_probe = ast2600_espi_vw_probe,
	.espi_vw_remove = ast2600_espi_vw_remove,
	.espi_oob_probe = ast2600_espi_oob_probe,
	.espi_oob_remove = ast2600_espi_oob_remove,
	.espi_flash_probe = ast2600_espi_flash_probe,
	.espi_flash_remove = ast2600_espi_flash_remove,
	.espi_isr = ast2600_espi_isr,
};

static const struct aspeed_espi_ops aspeed_espi_ast2700_ops = {
	.platform_id = ASPEED_ESPI_PLATFORM_ID_AST2700,
	.espi_pre_init = ast2700_espi_pre_init,
	.espi_post_init = ast2700_espi_post_init,
	.espi_deinit = ast2700_espi_deinit,
	.espi_perif_probe = ast2700_espi_perif_probe,
	.espi_perif_remove = ast2700_espi_perif_remove,
	.espi_vw_probe = ast2700_espi_vw_probe,
	.espi_vw_remove = ast2700_espi_vw_remove,
	.espi_oob_probe = ast2700_espi_oob_probe,
	.espi_oob_remove = ast2700_espi_oob_remove,
	.espi_flash_probe = ast2700_espi_flash_probe,
	.espi_flash_remove = ast2700_espi_flash_remove,
	.espi_isr = ast2700_espi_isr,
};

static const struct of_device_id aspeed_espi_of_matches[] = {
	{ .compatible = "aspeed,ast2500-espi", .data = &aspeed_espi_ast2500_ops },
	{ .compatible = "aspeed,ast2600-espi", .data = &aspeed_espi_ast2600_ops },
	{ .compatible = "aspeed,ast2700-espi", .data = &aspeed_espi_ast2700_ops },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_espi_of_matches);

static int aspeed_espi_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct aspeed_espi *espi;
	struct resource *res;
	int rc;

	espi = devm_kzalloc(dev, sizeof(*espi), GFP_KERNEL);
	if (!espi)
		return -ENOMEM;

	espi->dev = dev;
	match = of_match_device(aspeed_espi_of_matches, dev);
	if (!match)
		return -ENODEV;
	espi->ops = match->data;

	espi->pdev = pdev;

	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (rc) {
		dev_err(dev, "cannot set 64-bits DMA mask\n");
		return rc;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "cannot get resource\n");
		return -ENODEV;
	}

	espi->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(espi->regs)) {
		dev_err(dev, "cannot map registers\n");
		return PTR_ERR(espi->regs);
	}

	espi->irq = platform_get_irq(pdev, 0);
	if (espi->irq < 0) {
		dev_err(dev, "cannot get IRQ number\n");
		return -ENODEV;
	}

	espi->rst = devm_reset_control_get_optional(&pdev->dev, NULL);
	if (IS_ERR(espi->rst)) {
		dev_err(dev, "cannot get reset control\n");
		return PTR_ERR(espi->rst);
	}

	espi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(espi->clk)) {
		dev_err(dev, "cannot get clock control\n");
		return PTR_ERR(espi->clk);
	}

	rc = clk_prepare_enable(espi->clk);
	if (rc) {
		dev_err(dev, "cannot enable clocks\n");
		return rc;
	}

	espi->ops->espi_pre_init(espi);

	rc = espi->ops->espi_perif_probe(espi);
	if (rc) {
		dev_err(dev, "cannot init peripheral channel, rc=%d\n", rc);
		return rc;
	}

	rc = espi->ops->espi_vw_probe(espi);
	if (rc) {
		dev_err(dev, "cannot init vw channel, rc=%d\n", rc);
		goto err_remove_perif;
	}

	rc = espi->ops->espi_oob_probe(espi);
	if (rc) {
		dev_err(dev, "cannot init oob channel, rc=%d\n", rc);
		goto err_remove_vw;
	}

	rc = espi->ops->espi_flash_probe(espi);
	if (rc) {
		dev_err(dev, "cannot init flash channel, rc=%d\n", rc);
		goto err_remove_oob;
	}

	rc = devm_request_irq(dev, espi->irq, espi->ops->espi_isr, 0,
			      dev_name(dev), espi);
	if (rc) {
		dev_err(dev, "cannot request IRQ\n");
		goto err_remove_flash;
	}

	if (espi->perif.mmbi.enable) {
		rc = devm_request_irq(dev, espi->perif.mmbi.irq,
				      espi->perif.mmbi.mmbi_isr, 0, dev_name(dev),
				      espi);
		if (rc) {
			dev_err(dev, "cannot request MMBI IRQ\n");
			goto err_remove_flash;
		}
	}

	espi->ops->espi_post_init(espi);

	platform_set_drvdata(pdev, espi);

	dev_info(dev, "module loaded\n");

	return 0;

err_remove_flash:
	espi->ops->espi_flash_remove(espi);
err_remove_oob:
	espi->ops->espi_oob_remove(espi);
err_remove_vw:
	espi->ops->espi_vw_remove(espi);
err_remove_perif:
	espi->ops->espi_perif_remove(espi);

	return rc;
}

static void aspeed_espi_remove(struct platform_device *pdev)
{
	struct aspeed_espi *espi;
	struct device *dev;

	dev = &pdev->dev;

	espi = dev_get_drvdata(dev);

	espi->ops->espi_deinit(espi);

	if (espi->ops->espi_perif_remove(espi))
		dev_warn(dev, "cannot remove peripheral channel\n");
	if (espi->ops->espi_vw_remove(espi))
		dev_warn(dev, "cannot remove vw channel\n");
	if (espi->ops->espi_oob_remove(espi))
		dev_warn(dev, "cannot remove oob channel\n");
	if (espi->ops->espi_flash_remove(espi))
		dev_warn(dev, "cannot remove flash channel\n");
}

static struct platform_driver aspeed_espi_driver = {
	.driver = {
		.name = "aspeed-espi",
		.of_match_table = aspeed_espi_of_matches,
	},
	.probe = aspeed_espi_probe,
	.remove = aspeed_espi_remove,
};

module_platform_driver(aspeed_espi_driver);

MODULE_AUTHOR("Aspeed Technology Inc.");
MODULE_DESCRIPTION("Aspeed eSPI controller (AST2500/2600/2700)");
MODULE_LICENSE("GPL");
