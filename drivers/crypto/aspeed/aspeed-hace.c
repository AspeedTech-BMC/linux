// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 */

#include "aspeed-hace.h"
#include <crypto/engine.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#ifdef CONFIG_CRYPTO_DEV_ASPEED_DEBUG
#define HACE_DBG(d, fmt, ...)	\
	dev_info((d)->dev, "%s() " fmt, __func__, ##__VA_ARGS__)
#else
#define HACE_DBG(d, fmt, ...)	\
	dev_dbg((d)->dev, "%s() " fmt, __func__, ##__VA_ARGS__)
#endif

static unsigned char *dummy_key1;
static unsigned char *dummy_key2;

int find_dummy_key(const char *key, int keylen)
{
	int ret = 0;

	if (dummy_key1 && memcmp(key, dummy_key1, keylen) == 0)
		ret = 1;
	else if (dummy_key2 && memcmp(key, dummy_key2, keylen) == 0)
		ret = 2;

	return ret;
}

/* HACE interrupt service routine */
static irqreturn_t aspeed_hace_irq(int irq, void *dev)
{
	struct aspeed_hace_dev *hace_dev = (struct aspeed_hace_dev *)dev;
	struct aspeed_engine_crypto *crypto_engine = &hace_dev->crypto_engine;
	struct aspeed_engine_hash *hash_engine = &hace_dev->hash_engine;
	u32 sts;

	sts = ast_hace_read(hace_dev, ASPEED_HACE_STS);
	ast_hace_write(hace_dev, sts, ASPEED_HACE_STS);

	HACE_DBG(hace_dev, "irq status: 0x%x\n", sts);

	if (sts & HACE_HASH_ISR) {
		if (hash_engine->flags & CRYPTO_FLAGS_BUSY)
			tasklet_schedule(&hash_engine->done_task);
		else
			dev_warn(hace_dev->dev, "HASH no active requests.\n");
	}

	if (sts & HACE_CRYPTO_ISR) {
		if (crypto_engine->flags & CRYPTO_FLAGS_BUSY)
			tasklet_schedule(&crypto_engine->done_task);
		else
			dev_warn(hace_dev->dev, "CRYPTO no active requests.\n");
	}

	HACE_DBG(hace_dev, "handled\n");

	return IRQ_HANDLED;
}

static void aspeed_hace_register(struct aspeed_hace_dev *hace_dev)
{
#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_HASH
	aspeed_register_hace_hash_algs(hace_dev);
#endif
#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_CRYPTO
	aspeed_register_hace_crypto_algs(hace_dev);
#endif
}

static void aspeed_hace_unregister(struct aspeed_hace_dev *hace_dev)
{
#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_HASH
	aspeed_unregister_hace_hash_algs(hace_dev);
#endif
#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_CRYPTO
	aspeed_unregister_hace_crypto_algs(hace_dev);
#endif
}

static const struct of_device_id aspeed_hace_of_matches[] = {
	{ .compatible = "aspeed,ast2500-hace", .data = (void *)5, },
	{ .compatible = "aspeed,ast2600-hace", .data = (void *)6, },
	{ .compatible = "aspeed,ast2700-hace", .data = (void *)7, },
	{},
};

static int aspeed_hace_probe(struct platform_device *pdev)
{
	const struct of_device_id *hace_dev_id;
	struct aspeed_hace_dev *hace_dev;
	struct device_node *sec_node;
	struct device *dev = &pdev->dev;
	int rc;
	int err;

	hace_dev = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_hace_dev),
				GFP_KERNEL);
	if (!hace_dev)
		return -ENOMEM;

	hace_dev_id = of_match_device(aspeed_hace_of_matches, &pdev->dev);
	if (!hace_dev_id) {
		dev_err(&pdev->dev, "Failed to match hace dev id\n");
		return -EINVAL;
	}

	hace_dev->dev = &pdev->dev;
	hace_dev->version = (unsigned long)hace_dev_id->data;

	platform_set_drvdata(pdev, hace_dev);

	hace_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(hace_dev->regs))
		return PTR_ERR(hace_dev->regs);

	/* Get irq number and register it */
	hace_dev->irq = platform_get_irq(pdev, 0);
	if (hace_dev->irq < 0)
		return -ENXIO;

	rc = devm_request_irq(&pdev->dev, hace_dev->irq, aspeed_hace_irq, 0,
			      dev_name(&pdev->dev), hace_dev);
	if (rc) {
		dev_err(&pdev->dev, "Failed to request interrupt\n");
		return rc;
	}

	/* Get clk and enable it */
	hace_dev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(hace_dev->clk)) {
		dev_err(&pdev->dev, "Failed to get clk\n");
		return -ENODEV;
	}

	rc = clk_prepare_enable(hace_dev->clk);
	if (rc) {
		dev_err(&pdev->dev, "Failed to enable clock 0x%x\n", rc);
		return rc;
	}

	hace_dev->rst = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(hace_dev->rst)) {
		dev_err(&pdev->dev, "Failed to get hace reset\n");
		return PTR_ERR(hace_dev->rst);
	}

	rc = reset_control_deassert(hace_dev->rst);
	if (rc) {
		dev_err(&pdev->dev, "Deassert hace reset failed\n");
		return rc;
	}

	rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (rc) {
		dev_warn(&pdev->dev, "No suitable DMA available\n");
		return rc;
	}

#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_HASH
	rc = aspeed_hace_hash_init(hace_dev);
	if (rc) {
		dev_err(&pdev->dev, "Hash init failed\n");
		return rc;
	}
#endif
#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_CRYPTO
	rc = aspeed_hace_crypto_init(hace_dev);
	if (rc) {
		dev_err(&pdev->dev, "Crypto init failed\n");
		return rc;
	}

	if (of_find_property(dev->of_node, "dummy-key1", NULL)) {
		dummy_key1 = kzalloc(DUMMY_KEY_SIZE, GFP_KERNEL);
		if (dummy_key1) {
			err = of_property_read_u8_array(dev->of_node, "dummy-key1", dummy_key1, DUMMY_KEY_SIZE);
			if (err)
				dev_err(dev, "error of reading dummy_key 1\n");
		} else {
			dev_err(dev, "error dummy_key1 allocation\n");
		}
	}

	if (of_find_property(dev->of_node, "dummy-key2", NULL)) {
		dummy_key2 = kzalloc(DUMMY_KEY_SIZE, GFP_KERNEL);
		if (dummy_key2) {
			err = of_property_read_u8_array(dev->of_node, "dummy-key2", dummy_key2, DUMMY_KEY_SIZE);
			if (err)
				dev_err(dev, "error of reading dummy_key 2\n");
		} else {
			dev_err(dev, "error dummy_key2 allocation\n");
		}
	}

	sec_node = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-sbc");
	if (!sec_node) {
		dev_err(dev, "cannot find sbc node\n");
	} else {
		hace_dev->sec_regs = of_iomap(sec_node, 0);
		if (!hace_dev->sec_regs)
			dev_err(dev, "failed to map SBC registers\n");
	}
#endif
	aspeed_hace_register(hace_dev);

	dev_info(&pdev->dev, "Aspeed Crypto Accelerator successfully registered\n");

	return 0;
}

static int aspeed_hace_remove(struct platform_device *pdev)
{
	struct aspeed_hace_dev *hace_dev = platform_get_drvdata(pdev);
	struct aspeed_engine_crypto *crypto_engine = &hace_dev->crypto_engine;
	struct aspeed_engine_hash *hash_engine = &hace_dev->hash_engine;

	aspeed_hace_unregister(hace_dev);

#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_HASH
	crypto_engine_exit(hace_dev->crypt_engine_hash);
	tasklet_kill(&hash_engine->done_task);
#endif
#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_CRYPTO
	crypto_engine_exit(hace_dev->crypt_engine_crypto);
	tasklet_kill(&crypto_engine->done_task);
#endif
	clk_disable_unprepare(hace_dev->clk);

	return 0;
}

MODULE_DEVICE_TABLE(of, aspeed_hace_of_matches);

static struct platform_driver aspeed_hace_driver = {
	.probe		= aspeed_hace_probe,
	.remove		= aspeed_hace_remove,
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_hace_of_matches,
	},
};

module_platform_driver(aspeed_hace_driver);

MODULE_AUTHOR("Neal Liu <neal_liu@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed HACE driver Crypto Accelerator");
MODULE_LICENSE("GPL");
