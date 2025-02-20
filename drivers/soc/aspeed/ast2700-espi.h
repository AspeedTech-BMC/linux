/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 Aspeed Technology Inc.
 */
#ifndef _AST2700_ESPI_H_
#define _AST2700_ESPI_H_

#include <linux/bits.h>
#include "aspeed-espi-comm.h"

/* global registers */
#define ESPI_CTRL			0x000
#define ESPI_STS			0x004
#define ESPI_INT_STS			0x008
#define   ESPI_INT_STS_RST_DEASSERT	BIT(31)
#define   ESPI_INT_STS_RST_ASSERT	BIT(30)
#define   ESPI_INT_STS_CH3		BIT(3)
#define   ESPI_INT_STS_CH2		BIT(2)
#define   ESPI_INT_STS_CH1		BIT(1)
#define   ESPI_INT_STS_CH0		BIT(0)
#define ESPI_INT_EN			0x00c
#define   ESPI_INT_EN_RST_DEASSERT	BIT(31)
#define   ESPI_INT_EN_RST_ASSERT	BIT(30)
#define ESPI_DEV_ID			0x010
#define ESPI_CAP_GEN			0x014
#define ESPI_CAP_CH0			0x018
#define ESPI_CAP_CH1			0x01c
#define ESPI_CAP_CH2			0x020
#define ESPI_CAP_CH3_0			0x024
#define ESPI_CAP_CH3_1			0x028
#define ESPI_DEV_STS			0x030
#define ESPI_DBG_CTRL			0x034
#define ESPI_DBG_ADDRL			0x038
#define ESPI_DBG_ADDRH			0x03c
#define ESPI_DBG_CMD			0x040
#define ESPI_DBG_RES			0x044
#define ESPI_CH_ACC_CTRL		0x04c
#define ESPI_CH_ACC_OFST1		0x050
#define ESPI_CH_ACC_OFST2		0x054
#define ESPI_WPROT0			0x0f8
#define ESPI_WPROT1			0x0fc

/* peripheral channel (ch0) registers */
#define ESPI_CH0_CTRL			0x100
#define   ESPI_CH0_CTRL_NP_TX_RST	BIT(31)
#define   ESPI_CH0_CTRL_NP_RX_RST	BIT(30)
#define   ESPI_CH0_CTRL_PC_TX_RST	BIT(29)
#define   ESPI_CH0_CTRL_PC_RX_RST	BIT(28)
#define   ESPI_CH0_CTRL_NP_TX_DMA_EN	BIT(19)
#define   ESPI_CH0_CTRL_PC_TX_DMA_EN	BIT(17)
#define   ESPI_CH0_CTRL_PC_RX_DMA_EN	BIT(16)
#define   ESPI_CH0_CTRL_MCYC_RD_DIS_WDT	BIT(9)
#define   ESPI_CH0_CTRL_MCYC_WR_DIS_WDT	BIT(8)
#define   ESPI_CH0_CTRL_MCYC_RD_DIS	BIT(6)
#define   ESPI_CH0_CTRL_MCYC_WR_DIS	BIT(4)
#define   ESPI_CH0_CTRL_SW_RDY		BIT(1)
#define ESPI_CH0_STS			0x104
#define ESPI_CH0_INT_STS		0x108
#define   ESPI_CH0_INT_STS_PC_RX_CMPLT	BIT(0)
#define ESPI_CH0_INT_EN			0x10c
#define   ESPI_CH0_INT_EN_PC_RX_CMPLT	BIT(0)
#define ESPI_CH0_PC_RX_DMAL		0x110
#define ESPI_CH0_PC_RX_DMAH		0x114
#define ESPI_CH0_PC_RX_CTRL		0x118
#define   ESPI_CH0_PC_RX_CTRL_SERV_PEND	BIT(31)
#define   ESPI_CH0_PC_RX_CTRL_LEN	GENMASK(23, 12)
#define   ESPI_CH0_PC_RX_CTRL_TAG	GENMASK(11, 8)
#define   ESPI_CH0_PC_RX_CTRL_CYC	GENMASK(7, 0)
#define ESPI_CH0_PC_RX_DATA		0x11c
#define ESPI_CH0_PC_TX_DMAL		0x120
#define ESPI_CH0_PC_TX_DMAH		0x124
#define ESPI_CH0_PC_TX_CTRL		0x128
#define   ESPI_CH0_PC_TX_CTRL_TRIG_PEND	BIT(31)
#define   ESPI_CH0_PC_TX_CTRL_LEN	GENMASK(23, 12)
#define   ESPI_CH0_PC_TX_CTRL_TAG	GENMASK(11, 8)
#define   ESPI_CH0_PC_TX_CTRL_CYC	GENMASK(7, 0)
#define ESPI_CH0_PC_TX_DATA		0x12c
#define ESPI_CH0_NP_TX_DMAL		0x130
#define ESPI_CH0_NP_TX_DMAH		0x134
#define ESPI_CH0_NP_TX_CTRL		0x138
#define   ESPI_CH0_NP_TX_CTRL_TRIG_PEND	BIT(31)
#define   ESPI_CH0_NP_TX_CTRL_LEN	GENMASK(23, 12)
#define   ESPI_CH0_NP_TX_CTRL_TAG	GENMASK(11, 8)
#define   ESPI_CH0_NP_TX_CTRL_CYC	GENMASK(7, 0)
#define ESPI_CH0_NP_TX_DATA		0x13c
#define ESPI_CH0_MCYC0_SADDRL		0x140
#define ESPI_CH0_MCYC0_SADDRH		0x144
#define ESPI_CH0_MCYC0_TADDRL		0x148
#define ESPI_CH0_MCYC0_TADDRH		0x14c
#define ESPI_CH0_MCYC0_MASKL		0x150
#define   ESPI_CH0_MCYC0_MASKL_EN	BIT(0)
#define ESPI_CH0_MCYC0_MASKH		0x154
#define ESPI_CH0_MCYC1_SADDRL		0x158
#define ESPI_CH0_MCYC1_SADDRH		0x15c
#define ESPI_CH0_MCYC1_TADDRL		0x160
#define ESPI_CH0_MCYC1_TADDRH		0x164
#define ESPI_CH0_MCYC1_MASKL		0x168
#define   ESPI_CH0_MCYC1_MASKL_EN	BIT(0)
#define ESPI_CH0_MCYC1_MASKH		0x16c
#define ESPI_CH0_WPROT0			0x1f8
#define ESPI_CH0_WPROT1			0x1fc

/* virtual wire channel (ch1) registers */
#define ESPI_CH1_CTRL			0x200
#define   ESPI_CH1_CTRL_GPIO_HW		BIT(9)
#define   ESPI_CH1_CTRL_SW_RDY		BIT(1)
#define ESPI_CH1_STS			0x204
#define ESPI_CH1_INT_STS		0x208
#define   ESPI_CH1_INT_STS_GPIO		BIT(2)
#define ESPI_CH1_INT_EN			0x20c
#define   ESPI_CH1_INT_EN_GPIO		BIT(2)
#define ESPI_CH1_EVT0			0x210
#define ESPI_CH1_EVT0_INT_EN		0x214
#define ESPI_CH1_EVT0_INT_T0		0x218
#define ESPI_CH1_EVT0_INT_T1		0x21c
#define ESPI_CH1_EVT0_INT_T2		0x220
#define ESPI_CH1_EVT0_INT_STS		0x224
#define ESPI_CH1_EVT1			0x230
#define ESPI_CH1_EVT1_INT_EN		0x234
#define ESPI_CH1_EVT1_INT_T0		0x238
#define ESPI_CH1_EVT1_INT_T1		0x23c
#define ESPI_CH1_EVT1_INT_T2		0x240
#define ESPI_CH1_EVT1_INT_STS		0x244
#define ESPI_CH1_GPIO_VAL0		0x250
#define ESPI_CH1_GPIO_VAL1		0x254
#define ESPI_CH1_GPIO_DIR0		0x258
#define ESPI_CH1_GPIO_DIR1		0x258
#define ESPI_CH1_GPIO_RSTSEL0		0x260
#define ESPI_CH1_GPIO_RSTSEL1		0x264
#define ESPI_CH1_GPIO_GRP		0x268
#define ESPI_CH1_GP50_DIR0		0x270
#define ESPI_CH1_GP50_DIR1		0x274
#define ESPI_CH1_GP50_VAL0		0x278
#define ESPI_CH1_GP50_VAL1		0x27c
#define ESPI_CH1_SW_INT			0x280
#define ESPI_CH1_INT_RSTSEL0		0x284
#define ESPI_CH1_INT_RSTSEL1		0x288
#define ESPI_CH1_WPROT0			0x2f8
#define ESPI_CH1_WPROT1			0x2fc

/* out-of-band channel (ch2) registers */
#define ESPI_CH2_CTRL			0x300
#define   ESPI_CH2_CTRL_TX_RST		BIT(31)
#define   ESPI_CH2_CTRL_RX_RST		BIT(30)
#define   ESPI_CH2_CTRL_TX_DMA_EN	BIT(17)
#define   ESPI_CH2_CTRL_RX_DMA_EN	BIT(16)
#define   ESPI_CH2_CTRL_SW_RDY		BIT(4)
#define ESPI_CH2_STS			0x304
#define ESPI_CH2_INT_STS		0x308
#define   ESPI_CH2_INT_STS_RX_CMPLT	BIT(0)
#define ESPI_CH2_INT_EN			0x30c
#define   ESPI_CH2_INT_EN_RX_CMPLT	BIT(0)
#define ESPI_CH2_RX_DMAL		0x310
#define ESPI_CH2_RX_DMAH		0x314
#define ESPI_CH2_RX_CTRL		0x318
#define   ESPI_CH2_RX_CTRL_SERV_PEND	BIT(31)
#define   ESPI_CH2_RX_CTRL_PEC		BIT(24)
#define   ESPI_CH2_RX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH2_RX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH2_RX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_CH2_RX_DATA		0x31c
#define ESPI_CH2_TX_DMAL		0x320
#define ESPI_CH2_TX_DMAH		0x324
#define ESPI_CH2_TX_CTRL		0x328
#define   ESPI_CH2_TX_CTRL_TRIG_PEND	BIT(31)
#define   ESPI_CH2_TX_CTRL_PEC		BIT(24)
#define   ESPI_CH2_TX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH2_TX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH2_TX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_CH2_TX_DATA		0x32c
#define ESPI_CH2_RX_DESC_EPTR		0x330
#define ESPI_CH2_RX_DESC_RPTR		0x334
#define	  ESPI_CH2_RX_DESC_RPTR_UPDATE	BIT(31)
#define   ESPI_CH2_RX_DESC_RPTR_RP	GENMASK(11, 0)
#define ESPI_CH2_RX_DESC_WPTR		0x338
#define   ESPI_CH2_RX_DESC_WPTR_VALID	BIT(31)
#define   ESPI_CH2_RX_DESC_WPTR_SP	GENMASK(27, 16)
#define   ESPI_CH2_RX_DESC_WPTR_WP	GENMASK(11, 0)
#define ESPI_CH2_RX_DESC_TMOUT		0x33c
#define ESPI_CH2_TX_DESC_EPTR		0x340
#define ESPI_CH2_TX_DESC_RPTR		0x344
#define   ESPI_CH2_TX_DESC_RPTR_UPT	BIT(31)
#define ESPI_CH2_TX_DESC_WPTR		0x348
#define   ESPI_CH2_TX_DESC_WPTR_VALID	BIT(31)
#define ESPI_CH2_WPROT0			0x3f8
#define ESPI_CH2_WPROT1			0x3fc

/* flash channel (ch3) registers */
#define ESPI_CH3_CTRL			0x400
#define   ESPI_CH3_CTRL_TX_RST		BIT(31)
#define   ESPI_CH3_CTRL_RX_RST		BIT(30)
#define   ESPI_CH3_CTRL_TX_DMA_EN	BIT(17)
#define   ESPI_CH3_CTRL_RX_DMA_EN	BIT(16)
#define   ESPI_CH3_CTRL_EDAF_MODE	GENMASK(9, 8)
#define   ESPI_CH3_CTRL_SW_RDY		BIT(5)
#define ESPI_CH3_STS			0x404
#define ESPI_CH3_INT_STS		0x408
#define   ESPI_CH3_INT_STS_RX_CMPLT	BIT(0)
#define ESPI_CH3_INT_EN			0x40c
#define   ESPI_CH3_INT_EN_RX_CMPLT	BIT(0)
#define ESPI_CH3_RX_DMAL		0x410
#define ESPI_CH3_RX_DMAH		0x414
#define ESPI_CH3_RX_CTRL		0x418
#define   ESPI_CH3_RX_CTRL_SERV_PEND	BIT(31)
#define   ESPI_CH3_RX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH3_RX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH3_RX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_CH3_RX_DATA		0x41c
#define ESPI_CH3_TX_DMAL		0x420
#define ESPI_CH3_TX_DMAH		0x424
#define ESPI_CH3_TX_CTRL		0x428
#define   ESPI_CH3_TX_CTRL_TRIG_PEND	BIT(31)
#define   ESPI_CH3_TX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH3_TX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH3_TX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_CH3_TX_DATA		0x42c
#define ESPI_CH3_EDAF_TADDRL		0x430
#define ESPI_CH3_EDAF_TADDRH		0x434
#define ESPI_CH3_EDAF_MASKL		0x438
#define ESPI_CH3_EDAF_MASKH		0x43c
#define ESPI_CH3_WPROT0			0x4f8
#define ESPI_CH3_WPROT1			0x4fc

/* eDAF filter registers */
#define ESPI_EDAF_FLTR_SADDR0		0x510
#define ESPI_EDAF_FLTR_EADDR0		0x514
#define ESPI_EDAF_FLTR_SADDR1		0x518
#define ESPI_EDAF_FLTR_EADDR1		0x51c
#define ESPI_EDAF_FLTR_SADDR2		0x520
#define ESPI_EDAF_FLTR_EADDR2		0x524
#define ESPI_EDAF_FLTR_SADDR3		0x528
#define ESPI_EDAF_FLTR_EADDR3		0x52c
#define ESPI_EDAF_FLTR_SADDR4		0x530
#define ESPI_EDAF_FLTR_EADDR4		0x534
#define ESPI_EDAF_FLTR_SADDR5		0x538
#define ESPI_EDAF_FLTR_EADDR5		0x53c
#define ESPI_EDAF_FLTR_SADDR6		0x540
#define ESPI_EDAF_FLTR_EADDR6		0x544
#define ESPI_EDAF_FLTR_SADDR7		0x548
#define ESPI_EDAF_FLTR_EADDR7		0x54c
#define ESPI_EDAF_FLTR_SADDR8		0x550
#define ESPI_EDAF_FLTR_EADDR8		0x554
#define ESPI_EDAF_FLTR_SADDR9		0x558
#define ESPI_EDAF_FLTR_EADDR9		0x55c
#define ESPI_EDAF_FLTR_SADDR10		0x560
#define ESPI_EDAF_FLTR_EADDR10		0x564
#define ESPI_EDAF_FLTR_SADDR11		0x568
#define ESPI_EDAF_FLTR_EADDR11		0x56c
#define ESPI_EDAF_FLTR_SADDR12		0x570
#define ESPI_EDAF_FLTR_EADDR12		0x574
#define ESPI_EDAF_FLTR_SADDR13		0x578
#define ESPI_EDAF_FLTR_EADDR13		0x57c
#define ESPI_EDAF_FLTR_SADDR14		0x580
#define ESPI_EDAF_FLTR_EADDR14		0x584
#define ESPI_EDAF_FLTR_SADDR15		0x588
#define ESPI_EDAF_FLTR_EADDR15		0x58c
#define ESPI_EDAF_WPROT0		0x5f8
#define ESPI_EDAF_WPROT1		0x5fc

/* MMBI registers */
#define ESPI_MMBI_CTRL			0x800
#define   ESPI_MMBI_CTRL_INST_NUM	GENMASK(6, 4)
#define   ESPI_MMBI_CTRL_EN		BIT(0)
#define ESPI_MMBI_INT_STS		0x808
#define ESPI_MMBI_INT_EN		0x80c
#define ESPI_MMBI_HOST_RWP(x)		(0x810 + ((x) << 3))

enum ast2700_edaf_mode {
	EDAF_MODE_MIX,
	EDAF_MODE_SW,
	EDAF_MODE_HW,
	EDAF_MODES,
};

#endif
