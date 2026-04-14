/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2025 Aspeed Technology Inc.
 */
#ifndef _AST2700_ESPI_MMBI_H_
#define _AST2700_ESPI_MMBI_H_

/* MMBI Control registers */
#define ESPI_MMBI_CTRL			0x00
#define   ESPI_MMBI_CTRL_INST_NUM	GENMASK(6, 4)
#define   ESPI_MMBI_CTRL_EN			BIT(0)
#define ESPI_MMBI_INT_STATUS	0x08
#define   ESPI_MMBI_HOST_RWS0_INT	BIT(0)
#define   ESPI_MMBI_HOST_RWS1_INT	BIT(1)
#define ESPI_MMBI_INT_ENABLE	0x0C

#define ESPI_MMBI_HOST_RWP0_VAL	0x10
#define ESPI_MMBI_HOST_RWP1_VAL	0x14

#define ESPI_MMBI_HOST_READ_RWP0(idx) (((idx) * 8) + ESPI_MMBI_HOST_RWP0_VAL)
#define ESPI_MMBI_HOST_READ_RWP1(idx) (((idx) * 8) + ESPI_MMBI_HOST_RWP1_VAL)

struct aspeed_mmbi_get_empty_space {
	__u32 length;
};

struct aspeed_mmbi_get_config {
	bool h_rdy;
	__u32 h2b_wp;
	__u32 b2h_rp;
	__u32 h2b_rp;
	__u32 b2h_wp;
};

#define __ASPEED_MMBI_CTRL_IOCTL_MAGIC 0xbb
/*
 * This IOCTL is meant to read empty space in B2H buffer
 * in a specific channel
 */
#define ASPEED_MMBI_CTRL_IOCTL_GET_B2H_EMPTY_SPACE                             \
	_IOWR(__ASPEED_MMBI_CTRL_IOCTL_MAGIC, 0x00,                            \
	      struct aspeed_mmbi_get_empty_space)

/* This IOCTL to send BMC reset request */
#define ASPEED_MMBI_CTRL_IOCTL_SEND_RESET_REQUEST                           \
	_IOW(__ASPEED_MMBI_CTRL_IOCTL_MAGIC, 0x01, int)

/* This IOCTL is to Get Config in HROP and HRWP */
#define ASPEED_MMBI_CTRL_IOCTL_GET_CONFIG                             \
	_IOWR(__ASPEED_MMBI_CTRL_IOCTL_MAGIC, 0x02,                                \
	      struct aspeed_mmbi_get_config)

#endif
