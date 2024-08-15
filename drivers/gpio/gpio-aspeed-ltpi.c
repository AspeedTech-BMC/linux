// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 ASPEED
 *
 * Author: Billy Tsai <billy_tsai@aspeedtech.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#define LTPI_GPIO_IRQ_STS_BASE 0x200
#define LTPI_GPIO_IRQ_STS_OFFSET(x) (LTPI_GPIO_IRQ_STS_BASE + (x) * 0x4)
#define LTPI_GPIO_CTRL_REG_BASE 0x0
#define LTPI_GPIO_CTRL_REG_OFFSET(x) (LTPI_GPIO_CTRL_REG_BASE + (x) * 0x4)
#define LTPI_GPIO_OUT_DATA BIT(0)
#define LTPI_GPIO_IRQ_EN BIT(2)
#define LTPI_GPIO_IRQ_TYPE0 BIT(3)
#define LTPI_GPIO_IRQ_TYPE1 BIT(4)
#define LTPI_GPIO_IRQ_TYPE2 BIT(5)
#define LTPI_GPIO_RST_TOLERANCE BIT(6)
#define LTPI_GPIO_IRQ_STS BIT(12)
#define LTPI_GPIO_IN_DATA BIT(13)

static inline u32 field_get(u32 _mask, u32 _val)
{
	return (((_val) & (_mask)) >> (ffs(_mask) - 1));
}

static inline u32 field_prep(u32 _mask, u32 _val)
{
	return (((_val) << (ffs(_mask) - 1)) & (_mask));
}

static inline void ast_write_bits(void __iomem *addr, u32 mask, u32 val)
{
	iowrite32((ioread32(addr) & ~(mask)) | field_prep(mask, val), addr);
}

static inline void ast_clr_bits(void __iomem *addr, u32 mask)
{
	iowrite32((ioread32(addr) & ~(mask)), addr);
}

struct aspeed_ltpi_gpio {
	struct gpio_chip chip;
	struct device *dev;
	raw_spinlock_t lock;
	void __iomem *base;
	int irq;
};

static int aspeed_ltpi_gpio_init_valid_mask(struct gpio_chip *gc,
					    unsigned long *valid_mask,
					    unsigned int ngpios)
{
	bitmap_set(valid_mask, 0, ngpios);
	return 0;
}

static void aspeed_ltpi_gpio_irq_init_valid_mask(struct gpio_chip *gc,
						 unsigned long *valid_mask,
						 unsigned int ngpios)
{
	unsigned int i;

	/* input GPIOs are even bits */
	for (i = 0; i < ngpios; i++) {
		if (i % 2)
			clear_bit(i, valid_mask);
	}
}

static bool aspeed_ltpi_gpio_is_input(unsigned int offset)
{
	return !(offset % 2);
}

static int aspeed_ltpi_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_ltpi_gpio *gpio = gpiochip_get_data(gc);
	void __iomem const *addr =
		gpio->base + LTPI_GPIO_CTRL_REG_OFFSET(offset >> 1);
	unsigned long flags;
	u32 mask;
	int rc = 0;

	raw_spin_lock_irqsave(&gpio->lock, flags);

	mask = aspeed_ltpi_gpio_is_input(offset) ? LTPI_GPIO_IN_DATA :
						   LTPI_GPIO_OUT_DATA;
	rc = !!(field_get(mask, ioread32(addr)));

	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return rc;
}

static int ltpi_gpio_set_value(struct gpio_chip *gc, unsigned int offset,
			       int val)
{
	struct aspeed_ltpi_gpio *gpio = gpiochip_get_data(gc);
	void __iomem *addr =
		gpio->base + LTPI_GPIO_CTRL_REG_OFFSET(offset >> 1);
	u32 reg = 0;

	if (aspeed_ltpi_gpio_is_input(offset))
		return -EINVAL;

	reg = ioread32(addr);

	if (val)
		reg |= LTPI_GPIO_OUT_DATA;
	else
		reg &= ~LTPI_GPIO_OUT_DATA;

	iowrite32(reg, addr);

	return 0;
}

static void aspeed_ltpi_gpio_set(struct gpio_chip *gc, unsigned int offset,
				 int val)
{
	struct aspeed_ltpi_gpio *gpio = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	ltpi_gpio_set_value(gc, offset, val);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
}

static int aspeed_ltpi_gpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
	return aspeed_ltpi_gpio_is_input(offset) ? 0 : -EINVAL;
}

static int aspeed_ltpi_gpio_dir_out(struct gpio_chip *gc, unsigned int offset,
				    int val)
{
	struct aspeed_ltpi_gpio *gpio = gpiochip_get_data(gc);
	unsigned long flags;
	int rc;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	rc = ltpi_gpio_set_value(gc, offset, val);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return rc;
}

static int aspeed_ltpi_gpio_get_direction(struct gpio_chip *gc,
					  unsigned int offset)
{
	return !!aspeed_ltpi_gpio_is_input(offset);
}

static void irqd_to_aspeed_ltpi_gpio_data(struct irq_data *d,
					  struct aspeed_ltpi_gpio **gpio,
					  int *offset)
{
	struct aspeed_ltpi_gpio *internal;

	*offset = irqd_to_hwirq(d);
	internal = irq_data_get_irq_chip_data(d);
	WARN_ON(!internal);

	*gpio = internal;
}

static void aspeed_ltpi_gpio_irq_ack(struct irq_data *d)
{
	struct aspeed_ltpi_gpio *gpio;
	unsigned long flags;
	void __iomem *status_addr;
	int offset;

	irqd_to_aspeed_ltpi_gpio_data(d, &gpio, &offset);

	status_addr = gpio->base + LTPI_GPIO_CTRL_REG_OFFSET(offset >> 1);

	raw_spin_lock_irqsave(&gpio->lock, flags);

	ast_write_bits(status_addr, LTPI_GPIO_IRQ_STS, 1);

	raw_spin_unlock_irqrestore(&gpio->lock, flags);
}

static void aspeed_ltpi_gpio_irq_set_mask(struct irq_data *d, bool set)
{
	struct aspeed_ltpi_gpio *gpio;
	unsigned long flags;
	void __iomem *addr;
	int offset;

	irqd_to_aspeed_ltpi_gpio_data(d, &gpio, &offset);
	addr = gpio->base + LTPI_GPIO_CTRL_REG_OFFSET(offset >> 1);

	/* Unmasking the IRQ */
	if (set)
		gpiochip_enable_irq(&gpio->chip, irqd_to_hwirq(d));

	raw_spin_lock_irqsave(&gpio->lock, flags);
	if (set)
		ast_write_bits(addr, LTPI_GPIO_IRQ_EN, 1);
	else
		ast_clr_bits(addr, LTPI_GPIO_IRQ_EN);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	/* Masking the IRQ */
	if (!set)
		gpiochip_disable_irq(&gpio->chip, irqd_to_hwirq(d));
}

static void aspeed_ltpi_gpio_irq_mask(struct irq_data *d)
{
	aspeed_ltpi_gpio_irq_set_mask(d, false);
}

static void aspeed_ltpi_gpio_irq_unmask(struct irq_data *d)
{
	aspeed_ltpi_gpio_irq_set_mask(d, true);
}

static int aspeed_ltpi_gpio_set_type(struct irq_data *d, unsigned int type)
{
	u32 type0 = 0;
	u32 type1 = 0;
	u32 type2 = 0;
	irq_flow_handler_t handler;
	struct aspeed_ltpi_gpio *gpio;
	unsigned long flags;
	void __iomem *addr;
	int offset;

	irqd_to_aspeed_ltpi_gpio_data(d, &gpio, &offset);
	addr = gpio->base + LTPI_GPIO_CTRL_REG_OFFSET(offset >> 1);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		type2 = 1;
		fallthrough;
	case IRQ_TYPE_EDGE_RISING:
		type0 = 1;
		fallthrough;
	case IRQ_TYPE_EDGE_FALLING:
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		type0 = 1;
		fallthrough;
	case IRQ_TYPE_LEVEL_LOW:
		type1 = 1;
		handler = handle_level_irq;
		break;
	default:
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&gpio->lock, flags);

	ast_write_bits(addr, LTPI_GPIO_IRQ_TYPE2, type2);
	ast_write_bits(addr, LTPI_GPIO_IRQ_TYPE1, type1);
	ast_write_bits(addr, LTPI_GPIO_IRQ_TYPE0, type0);

	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	irq_set_handler_locked(d, handler);
	return 0;
}

static void aspeed_ltpi_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_chip *ic = irq_desc_get_chip(desc);
	struct aspeed_ltpi_gpio const *gpio = gpiochip_get_data(gc);
	unsigned int i, p, banks;
	unsigned long reg;
	void __iomem const *addr;

	chained_irq_enter(ic, desc);

	banks = DIV_ROUND_UP(gpio->chip.ngpio >> 2, 32);
	for (i = 0; i < banks; i++) {
		addr = gpio->base + LTPI_GPIO_IRQ_STS_OFFSET(i);

		reg = ioread32(addr);

		for_each_set_bit(p, &reg, 32)
			generic_handle_domain_irq(gc->irq.domain,
						  (i * 32 + p) * 2);
	}
	chained_irq_exit(ic, desc);
}

static void aspeed_ltpi_gpio_irq_print_chip(struct irq_data *d,
					    struct seq_file *p)
{
	struct aspeed_ltpi_gpio *gpio;
	int offset;

	irqd_to_aspeed_ltpi_gpio_data(d, &gpio, &offset);
	seq_printf(p, dev_name(gpio->dev));
}

static const struct irq_chip aspeed_ltpi_gpio_irq_chip = {
	.irq_ack = aspeed_ltpi_gpio_irq_ack,
	.irq_mask = aspeed_ltpi_gpio_irq_mask,
	.irq_unmask = aspeed_ltpi_gpio_irq_unmask,
	.irq_set_type = aspeed_ltpi_gpio_set_type,
	.irq_print_chip = aspeed_ltpi_gpio_irq_print_chip,
	.flags = IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int aspeed_ltpi_gpio_setup_irqs(struct aspeed_ltpi_gpio *gpio,
				       struct platform_device *pdev)
{
	int rc;
	struct gpio_irq_chip *irq;

	rc = platform_get_irq(pdev, 0);
	if (rc < 0)
		return rc;

	gpio->irq = rc;

	irq = &gpio->chip.irq;
	gpio_irq_chip_set_chip(irq, &aspeed_ltpi_gpio_irq_chip);
	irq->init_valid_mask = aspeed_ltpi_gpio_irq_init_valid_mask;
	irq->handler = handle_bad_irq;
	irq->default_type = IRQ_TYPE_NONE;
	irq->parent_handler = aspeed_ltpi_gpio_irq_handler;
	irq->parent_handler_data = gpio;
	irq->parents = &gpio->irq;
	irq->num_parents = 1;

	return 0;
}

static int aspeed_ltpi_gpio_reset_tolerance(struct gpio_chip *chip,
					    unsigned int offset, bool enable)
{
	struct aspeed_ltpi_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;
	void __iomem *reg;

	reg = gpio->base + LTPI_GPIO_CTRL_REG_OFFSET(offset >> 1);

	raw_spin_lock_irqsave(&gpio->lock, flags);

	if (enable)
		ast_write_bits(reg, LTPI_GPIO_RST_TOLERANCE, 1);
	else
		ast_clr_bits(reg, LTPI_GPIO_RST_TOLERANCE);

	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int aspeed_ltpi_gpio_set_config(struct gpio_chip *chip,
				       unsigned int offset,
				       unsigned long config)
{
	unsigned long param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);

	if (param == PIN_CONFIG_PERSIST_STATE)
		return aspeed_ltpi_gpio_reset_tolerance(chip, offset, arg);

	return -EOPNOTSUPP;
}

static const struct of_device_id aspeed_ltpi_gpio_of_table[] = {
	{ .compatible = "aspeed,ast2700-ltpi-gpio" },
	{}
};

MODULE_DEVICE_TABLE(of, aspeed_ltpi_gpio_of_table);

static int __init aspeed_ltpi_gpio_probe(struct platform_device *pdev)
{
	u32 nr_gpios;
	struct aspeed_ltpi_gpio *gpio;
	int rc;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	gpio->dev = &pdev->dev;

	rc = device_property_read_u32(&pdev->dev, "ngpios", &nr_gpios);
	if (rc < 0) {
		dev_err(&pdev->dev, "Could not read ngpios property\n");
		return -EINVAL;
	}

	raw_spin_lock_init(&gpio->lock);

	gpio->chip.parent = &pdev->dev;
	gpio->chip.ngpio = nr_gpios * 2;
	gpio->chip.init_valid_mask = aspeed_ltpi_gpio_init_valid_mask;
	gpio->chip.direction_input = aspeed_ltpi_gpio_dir_in;
	gpio->chip.direction_output = aspeed_ltpi_gpio_dir_out;
	gpio->chip.get_direction = aspeed_ltpi_gpio_get_direction;
	gpio->chip.request = NULL;
	gpio->chip.free = NULL;
	gpio->chip.get = aspeed_ltpi_gpio_get;
	gpio->chip.set = aspeed_ltpi_gpio_set;
	gpio->chip.set_config = aspeed_ltpi_gpio_set_config;
	gpio->chip.label = dev_name(&pdev->dev);
	gpio->chip.base = -1;

	aspeed_ltpi_gpio_setup_irqs(gpio, pdev);

	rc = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
	if (rc < 0)
		return rc;

	return 0;
}

static struct platform_driver aspeed_ltpi_gpio_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_ltpi_gpio_of_table,
	},
};

module_platform_driver_probe(aspeed_ltpi_gpio_driver, aspeed_ltpi_gpio_probe);
MODULE_DESCRIPTION("Aspeed LTPI GPIO Driver");
