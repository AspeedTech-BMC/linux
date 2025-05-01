// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright ASPEED Technology

#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/reset-controller.h>

#include <dt-bindings/clock/aspeed,ast1800-clk.h>
#include <dt-bindings/reset/aspeed,ast1800-reset.h>

#define AST1800_CLK_25MHZ 25000000
#define AST1800_CLK_24MHZ 24000000
#define AST1800_CLK_192MHZ 192000000

#define AST1800_CLK_STOP1 0x50
#define AST1800_CLK_STOP2 0x54
#define AST1800_CLK_STOP3 0x58

#define AST1800_CLK_SEL	0x60

#define AST1800_HPLL_PARAM 0x70
#define AST1800_EPLL_PARAM 0x78
#define AST1800_LPLL_PARAM 0x80
#define AST1800_UXCLK_CTRL 0x88
#define AST1800_HUXCLK_CTRL 0x8C

static DEFINE_IDA(ast1800_clk_ida);

/* Globally visible clocks */
static DEFINE_SPINLOCK(ast1800_clk_lock);

static const struct clk_div_table ast1800_clk_div_table[] = {
	{ 0x0, 5 },
	{ 0x1, 10 },
	{ 0x2, 20 },
	{ 0x3, 40 },
};

static struct clk_hw *AST1800_calc_uclk(const char *name, u32 val)
{
	unsigned int mult, div;

	/* UARTCLK = UXCLK * R / (N * 2) */
	u32 r = val & 0xff;
	u32 n = (val >> 8) & 0x3ff;

	mult = r;
	div = n * 2;

	return clk_hw_register_fixed_factor(NULL, name, "ast1800-uxclk", 0, mult, div);
};

static struct clk_hw *AST1800_calc_huclk(const char *name, u32 val)
{
	unsigned int mult, div;

	/* UARTCLK = UXCLK * R / (N * 2) */
	u32 r = val & 0xff;
	u32 n = (val >> 8) & 0x3ff;

	mult = r;
	div = n * 2;

	return clk_hw_register_fixed_factor(NULL, name, "ast1800-huxclk", 0, mult, div);
};

struct clk_hw *AST1800_calc_pll(const char *name, const char *parent_name, u32 val)
{
	unsigned int mult, div;

	if (val & BIT(24)) {
		/* Pass through mode */
		mult = 1;
		div = 1;
	} else {
		/* F = 25Mhz * [(M + 1) / (n + 1)] / (p + 1) */
		u32 m = val & 0x1fff;
		u32 n = (val >> 13) & 0x3f;
		u32 p = (val >> 19) & 0xf;

		mult = (m + 1) / (n + 1);
		div = (p + 1);
	}
	return clk_hw_register_fixed_factor(NULL, name, parent_name, 0, mult, div);
};

static int AST1800_clk_is_enabled(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	u32 clk = BIT(gate->bit_idx % 32);
	u32 reg_offset;

	if (gate->bit_idx < 32)
		reg_offset = 0x50;
	else if (gate->bit_idx < 64)
		reg_offset = 0x54;
	else
		reg_offset = 0x58;

	u32 reg = readl(gate->reg + reg_offset);

	return !(reg & clk);
}

static int AST1800_clk_enable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	u32 clk = BIT(gate->bit_idx % 32);
	u32 reg;

	if (gate->bit_idx < 32)
		reg = 0x50;
	else if (gate->bit_idx < 64)
		reg = 0x54;
	else
		reg = 0x58;

	u32 val = readl(gate->reg + reg);

	if (val & clk)
		writel(val & ~clk, gate->reg + reg);

	return 0;
}

static void AST1800_clk_disable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	u32 clk = BIT(gate->bit_idx % 32);
	u32 reg;

	if (gate->bit_idx < 32)
		reg = 0x50;
	else if (gate->bit_idx < 64)
		reg = 0x54;
	else
		reg = 0x58;

	u32 val = readl(gate->reg + reg);

	if (!(val & clk))
		writel(val | clk, gate->reg + reg);
}

static const struct clk_ops AST1800_clk_gate_ops = {
	.enable = AST1800_clk_enable,
	.disable = AST1800_clk_disable,
	.is_enabled = AST1800_clk_is_enabled,
};

static struct clk_hw *AST1800_clk_hw_register_gate(struct device *dev, const char *name,
						   const char *parent_name, unsigned long flags,
						   void __iomem *reg, u8 clock_idx,
						   u8 clk_gate_flags, spinlock_t *lock)
{
	struct clk_gate *gate;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret = -EINVAL;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &AST1800_clk_gate_ops;
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	gate->reg = reg;
	gate->bit_idx = clock_idx;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->hw.init = &init;

	hw = &gate->hw;
	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(gate);
		hw = ERR_PTR(ret);
	}

	return hw;
}

struct ast1800_reset {
	void __iomem *base;
	struct reset_controller_dev rcdev;
};

#define to_rc_data(p) container_of(p, struct ast1800_reset, rcdev)

static int ast1800_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ast1800_reset *rc = to_rc_data(rcdev);
	u32 rst = BIT(id % 32);
	u32 reg = id >= 32 ? 0x44 : 0x40;

	writel(readl(rc->base + reg) | rst, rc->base + reg);
	return 0;
}

static int ast1800_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ast1800_reset *rc = to_rc_data(rcdev);
	u32 rst = BIT(id % 32);
	u32 reg = id >= 32 ? 0x44 : 0x40;

	/* Use set to clear register */
	writel(readl(rc->base + reg) & ~rst, rc->base + reg);
	return 0;
}

static int ast1800_reset_status(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ast1800_reset *rc = to_rc_data(rcdev);
	u32 rst = BIT(id % 32);
	u32 reg = id >= 32 ? 0x44 : 0x40;

	return (readl(rc->base + reg) & rst);
}

static const struct reset_control_ops ast1800_reset_ops = {
	.assert = ast1800_reset_assert,
	.deassert = ast1800_reset_deassert,
	.status = ast1800_reset_status,
};

static const char *const uxclk_sel0[] = {
	"ast1800_0-hpll_div5",
	"ast1800_0-hpll_div4",
	"ast1800_0-hpll_div2",
	"ast1800_0-epll_div4",
};

static const char *const uxclk_sel1[] = {
	"ast1800_1-hpll_div5",
	"ast1800_1-hpll_div4",
	"ast1800_1-hpll_div2",
	"ast1800_1-epll_div4",
};

static const char *const uartclk_sel0[] = {
	"ast1800_0-uartxclk",
	"ast1800_0-huartxclk",
};

static const char *const uartclk_sel1[] = {
	"ast1800_1-uartxclk",
	"ast1800_1-huartxclk",
};

static const char *const spiclk_sel0[] = {
	"ast1800_0-hpll",
	"ast1800_0-epll",
};

static const char *const spiclk_sel1[] = {
	"ast1800_1-hpll",
	"ast1800_1-epll",
};

static const char *const i3cclk_sel0[] = {
	"ast1800_0-hpll_div4",
	"ast1800_0-lpll",
};

static const char *const i3cclk_sel1[] = {
	"ast1800_1-hpll_div4",
	"ast1800_1-lpll",
};

#define CREATE_CLK_NAME(id, suffix) kasprintf(GFP_KERNEL, "ast1800_%d-%s", id, suffix)

static int AST1800_clk_init(struct device_node *ast1800_node)
{
	struct clk_hw_onecell_data *clk_data;
	struct ast1800_reset *reset;
	void __iomem *clk_base;
	struct clk_hw **clks;
	struct clk_hw *hw;
	u32 val;
	int ret;

	int id = ida_simple_get(&ast1800_clk_ida, 0, 0, GFP_KERNEL);

	clk_base = of_iomap(ast1800_node, 0);
	WARN_ON(!clk_base);

	clk_data = kzalloc(struct_size(clk_data, hws, AST1800_NUM_CLKS), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = AST1800_NUM_CLKS;
	clks = clk_data->hws;

	reset = kzalloc(sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;

	reset->base = clk_base;

	reset->rcdev.owner = THIS_MODULE;
	reset->rcdev.nr_resets = AST1800_RESET_NUMS;
	reset->rcdev.ops = &ast1800_reset_ops;
	reset->rcdev.of_node = ast1800_node;

	ret = reset_controller_register(&reset->rcdev);
	if (ret) {
		pr_err("soc1 failed to register reset controller\n");
		return ret;
	}
	/*
	 * Ast1800 A0 workaround:
	 * I3C reset should assert all of the I3C controllers simultaneously.
	 * Otherwise, it may lead to failure in accessing I3C registers.
	 */
	if (!(readl(clk_base) & BIT(16))) {
		for (int i = AST1800_RESET_I3C0; i <= AST1800_RESET_I3C15; i++)
			ast1800_reset_assert(&reset->rcdev, i);
	}

	hw = clk_hw_register_fixed_rate(NULL, CREATE_CLK_NAME(id, "clkin"),
					NULL, 0, AST1800_CLK_25MHZ);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	clks[AST1800_CLKIN] = hw;

	/* HPLL 1000Mhz */
	val = readl(clk_base + AST1800_HPLL_PARAM);
	clks[AST1800_CLK_HPLL] = AST1800_calc_pll(CREATE_CLK_NAME(id, "hpll"),
						  CREATE_CLK_NAME(id, "clkin"), val);

	/* EPLL 960Mhz */
	val = readl(clk_base + AST1800_EPLL_PARAM);
	clks[AST1800_CLK_EPLL] = AST1800_calc_pll(CREATE_CLK_NAME(id, "epll"),
						  CREATE_CLK_NAME(id, "clkin"), val);

	/* LPLL 1000Mhz */
	val = readl(clk_base + AST1800_LPLL_PARAM);
	clks[AST1800_CLK_LPLL] = AST1800_calc_pll(CREATE_CLK_NAME(id, "lpll"),
						  CREATE_CLK_NAME(id, "clkin"), val);

	clks[AST1800_CLK_HPLL_DIV2] =
		clk_hw_register_fixed_factor(NULL, CREATE_CLK_NAME(id, "hpll_div2"),
					     CREATE_CLK_NAME(id, "hpll"), 0, 1, 2);

	clks[AST1800_CLK_HPLL_DIV4] =
		clk_hw_register_fixed_factor(NULL, CREATE_CLK_NAME(id, "hpll_div4"),
					     CREATE_CLK_NAME(id, "hpll"), 0, 1, 4);

	clks[AST1800_CLK_HPLL_DIV5] =
		clk_hw_register_fixed_factor(NULL, CREATE_CLK_NAME(id, "hpll_div5"),
					     CREATE_CLK_NAME(id, "hpll"), 0, 1, 5);

	/* uxclk mux selection */
	clks[AST1800_CLK_UXCLK] =
		clk_hw_register_mux(NULL, CREATE_CLK_NAME(id, "uxclk"),
				    (id == 0) ? uxclk_sel0 : uxclk_sel1,
				    (id == 0) ? ARRAY_SIZE(uxclk_sel0) : ARRAY_SIZE(uxclk_sel1),
				    0, clk_base + AST1800_CLK_SEL,
				    8, 2, 0, &ast1800_clk_lock);

	val = readl(clk_base + AST1800_UXCLK_CTRL);
	clks[AST1800_CLK_UARTX] = AST1800_calc_uclk(CREATE_CLK_NAME(id, "uartxclk"), val);

	/* huxclk mux selection */
	clks[AST1800_CLK_HUXCLK] =
		clk_hw_register_mux(NULL, CREATE_CLK_NAME(id, "huxclk"),
				    (id == 0) ? uxclk_sel0 : uxclk_sel1,
				    (id == 0) ? ARRAY_SIZE(uxclk_sel0) : ARRAY_SIZE(uxclk_sel1),
				    0, clk_base + AST1800_CLK_SEL,
				    10, 2, 0, &ast1800_clk_lock);

	val = readl(clk_base + AST1800_HUXCLK_CTRL);
	clks[AST1800_CLK_HUARTX] = AST1800_calc_huclk(CREATE_CLK_NAME(id, "huartxclk"), val);

	/* AHB CLK = 200Mhz */
	clks[AST1800_CLK_AHB] =
		clk_hw_register_divider_table(NULL, CREATE_CLK_NAME(id, "ahb"),
					      CREATE_CLK_NAME(id, "hpll"),
					      0, clk_base + AST1800_CLK_SEL,
					      0, 2, 0, ast1800_clk_div_table, &ast1800_clk_lock);

	/* APB CLK = 100Mhz */
	clks[AST1800_CLK_APB] =
		clk_hw_register_divider_table(NULL, CREATE_CLK_NAME(id, "apb"),
					      CREATE_CLK_NAME(id, "hpll"),
					      0, clk_base + AST1800_CLK_SEL,
					      0, 2, 0, ast1800_clk_div_table, &ast1800_clk_lock);

	clks[AST1800_CLK_I3C] =
		clk_hw_register_mux(NULL, CREATE_CLK_NAME(id, "i3cclk"),
				    (id == 0) ? i3cclk_sel0 : i3cclk_sel1,
				    (id == 0) ? ARRAY_SIZE(i3cclk_sel0) : ARRAY_SIZE(i3cclk_sel1),
				    0, clk_base + AST1800_CLK_SEL,
				    12, 1, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C0] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c0clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     16, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C1] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c1clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     17, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C2] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c2clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     18, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C3] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c3clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     19, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C4] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c4clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     20, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C5] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c5clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     21, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C6] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c6clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     22, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C7] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c7clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     23, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C8] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c8clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     24, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C9] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c9clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     25, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C10] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c10clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     26, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C11] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c11clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     27, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C12] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c12clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     28, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C13] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c13clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     29, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C14] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c14clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     30, 0, &ast1800_clk_lock);

	clks[AST1800_CLK_GATE_I3C15] =
		AST1800_clk_hw_register_gate(NULL, CREATE_CLK_NAME(id, "i3c15clk-gate"),
					     CREATE_CLK_NAME(id, "i3cclk"),
					     0, clk_base + AST1800_CLK_STOP2,
					     31, 0, &ast1800_clk_lock);

	of_clk_add_hw_provider(ast1800_node, of_clk_hw_onecell_get, clk_data);

	return 0;
};

CLK_OF_DECLARE_DRIVER(ast1800, "aspeed,ast1800-scu", AST1800_clk_init);
