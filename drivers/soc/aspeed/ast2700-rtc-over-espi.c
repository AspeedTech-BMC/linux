// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Aspeed Technology Inc.
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/rtc.h>

#define SYNC_INTERVAL_MS_DEFAULT 1000

#define RTC_REG0		0x0
#define   RTC_REG0_MINUTES_ALARM GENMASK(31, 24)
#define   RTC_REG0_MINUTES	GENMASK(23, 16)
#define   RTC_REG0_SECONDS_ALARM GENMASK(15, 8)
#define   RTC_REG0_SECONDS	GENMASK(7, 0)
#define RTC_REG1		0x4
#define   RTC_REG1_DAY_OF_MONTH	GENMASK(31, 24)
#define   RTC_REG1_DAY_OF_WEEK	GENMASK(23, 16)
#define   RTC_REG1_HOURS_ALARM	GENMASK(15, 8)
#define   RTC_REG1_HOURS	GENMASK(7, 0)
#define RTC_REG2		0x8
#define   RTC_REG2_YEAR		GENMASK(15, 8)
#define   RTC_REG2_MONTH	GENMASK(7, 0)
#define RTC_REG_MAX		0xc

struct rtc_espi_sync {
	struct rtc_device *rtc_dev;
	struct device *dev;
	void __iomem *espi_ram_base;
	u32 interval_ms;
	struct delayed_work sync_work;
	struct workqueue_struct *wq;
};

static void sync_rtc_to_espi_work(struct work_struct *work)
{
	struct rtc_espi_sync *ctx = container_of(to_delayed_work(work), struct rtc_espi_sync, sync_work);
	struct rtc_time tm;
	u32 reg, sec, min, hour, day_of_week, day_of_month, month, year;
	u32 sec_alarm, min_alarm, hour_alarm;
	int ret;

	if (!ctx->espi_ram_base) {
		dev_err(ctx->dev, "eSPI RAM base not mapped\n");
		return;
	}

	ret = rtc_read_time(ctx->rtc_dev, &tm);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to read RTC time: %d\n", ret);
		goto reschedule;
	}
	dev_dbg(ctx->dev, "%s: tm data: secs=%d, mins=%d, hours=%d, mon=%d, year=%d\n",
		__func__, tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mon, tm.tm_year);
	dev_dbg(ctx->dev, "%s: tm data: wday(day of week)=%d, mday(day of month)=%d\n",
		__func__, tm.tm_wday, tm.tm_mday);

	/* Write the eSPI RAM registers */
	reg = FIELD_PREP(RTC_REG0_SECONDS, tm.tm_sec) |
	      FIELD_PREP(RTC_REG0_MINUTES, tm.tm_min);
	writel(reg, (ctx->espi_ram_base + RTC_REG0));

	reg = FIELD_PREP(RTC_REG1_HOURS, tm.tm_hour) |
	      FIELD_PREP(RTC_REG1_DAY_OF_MONTH, tm.tm_mday) |
	      FIELD_PREP(RTC_REG1_DAY_OF_WEEK, tm.tm_wday + 1);
	writel(reg, ctx->espi_ram_base + RTC_REG1);

	reg = FIELD_PREP(RTC_REG2_MONTH, tm.tm_mon + 1) |
	      FIELD_PREP(RTC_REG2_YEAR, tm.tm_year % 100);
	writel(reg, ctx->espi_ram_base + RTC_REG2);

	/* Read back the values to verify */
	reg = readl(ctx->espi_ram_base + RTC_REG0);
	sec = FIELD_GET(RTC_REG0_SECONDS, reg);
	sec_alarm = FIELD_GET(RTC_REG0_SECONDS_ALARM, reg);
	min = FIELD_GET(RTC_REG0_MINUTES, reg);
	min_alarm = FIELD_GET(RTC_REG0_MINUTES_ALARM, reg);

	reg = readl(ctx->espi_ram_base + RTC_REG1);
	hour = FIELD_GET(RTC_REG1_HOURS, reg);
	hour_alarm = FIELD_GET(RTC_REG1_HOURS_ALARM, reg);
	day_of_week = FIELD_GET(RTC_REG1_DAY_OF_WEEK, reg);
	day_of_month = FIELD_GET(RTC_REG1_DAY_OF_MONTH, reg);

	reg = readl(ctx->espi_ram_base + RTC_REG2);
	month = FIELD_GET(RTC_REG2_MONTH, reg);
	year = FIELD_GET(RTC_REG2_YEAR, reg);

	dev_dbg(ctx->dev, "%s: eSPI RAM data: secs=%d, mins=%d, hours=%d, ",
		__func__, sec, min, hour);
	dev_dbg(ctx->dev, "day_of_week=%d, day_of_month=%d, month=%d, year=%d\n",
		day_of_week, day_of_month, month, year);
	dev_dbg(ctx->dev, "%s: eSPI RAM data: secs_alarm=%d, mins_alarm=%d, hours_alarm=%d\n",
		__func__, sec_alarm, min_alarm, hour_alarm);

reschedule:
	queue_delayed_work(ctx->wq, &ctx->sync_work, msecs_to_jiffies(ctx->interval_ms));
}

static int rtc_espi_sync_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtc_espi_sync *ctx;
	struct device_node *np = dev->of_node;
	struct rtc_device *rtc;
	struct resource *res;
	u32 interval = SYNC_INTERVAL_MS_DEFAULT;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	/* Ensure the device tree node is valid */
	rtc = rtc_class_open("rtc0");
	if (!rtc) {
		dev_info(dev, "RTC not ready, deferring probe\n");
		return -EPROBE_DEFER;
	}

	/* Check if the RTC device supports the read_time operation */
	if (!rtc->ops || !rtc->ops->read_time) {
		dev_err(dev, "RTC device does not support read_time operation\n");
		rtc_class_close(rtc);
		return -ENODEV;
	}

	ctx->rtc_dev = rtc;
	ctx->dev = dev;

	/* Get eSPI RAM handle and map */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->espi_ram_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->espi_ram_base))
		return PTR_ERR(ctx->espi_ram_base);

	if (!ctx->espi_ram_base)
		memset(ctx->espi_ram_base, 0, RTC_REG_MAX);

	/* Optional: polling interval */
	of_property_read_u32(np, "interval-ms", &interval);
	ctx->interval_ms = interval;

	ctx->wq = alloc_workqueue("rtc_espi_sync_wq", WQ_UNBOUND, 0);
	if (!ctx->wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&ctx->sync_work, sync_rtc_to_espi_work);
	queue_delayed_work(ctx->wq, &ctx->sync_work, 0);

	dev_info(dev, "RTC-eSPI RAM sync initialized, every %u ms\n", ctx->interval_ms);

	platform_set_drvdata(pdev, ctx);
	return 0;
}

static void rtc_espi_sync_remove(struct platform_device *pdev)
{
	struct rtc_espi_sync *ctx = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&ctx->sync_work);
	destroy_workqueue(ctx->wq);

	if (ctx->rtc_dev)
		rtc_class_close(ctx->rtc_dev);
}

static const struct of_device_id rtc_espi_sync_of_match[] = {
	{ .compatible = "aspeed,ast2700-rtc-over-espi" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtc_espi_sync_of_match);

static struct platform_driver rtc_espi_sync_driver = {
	.driver = {
		.name = "ast2700-rtc-over-espi",
		.of_match_table = rtc_espi_sync_of_match,
	},
	.probe = rtc_espi_sync_probe,
	.remove = rtc_espi_sync_remove,
};
module_platform_driver(rtc_espi_sync_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Chen <kevin_chen@aspeedtech.com>");
MODULE_DESCRIPTION("RTC to eSPI RAM sync driver");
