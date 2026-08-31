/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2026 ASPEED Technology Inc.
 *
 * Caliptra-SS MCI (Manageability Controller Interface): low-level mailbox
 * register layout, direct status register layout, and the transport API
 * exported by drivers/soc/aspeed/aspeed-cptra-mci.c.
 *
 * This driver is deliberately command-agnostic: it does not know the
 * layout of any individual mailbox command's request/response, how its
 * checksum is computed, or what the command IDs mean. All of that lives in
 * whichever caller builds the request -- for the userspace ioctl path
 * (CPTRA_MCI_IOC_MBOX_EXECUTE / CPTRA_MCI_IOC_REG_READ, defined in
 * aspeed-cptra-mci.c), that's entirely in the userspace test tool.
 */

#ifndef _ASPEED_CPTRA_MCI_H
#define _ASPEED_CPTRA_MCI_H

#include <linux/bits.h>
#include <linux/types.h>

/* mcu_mbox0_csr register offsets, valid once the CSR page is selected */
#define CPTRA_MCI_MBOX_LOCK			0x00
#define CPTRA_MCI_MBOX_USER			0x04
#define CPTRA_MCI_MBOX_TARGET_USER		0x08
#define CPTRA_MCI_MBOX_TARGET_USER_VALID	0x0c
#define CPTRA_MCI_MBOX_CMD			0x10
#define CPTRA_MCI_MBOX_DLEN			0x14
#define CPTRA_MCI_MBOX_EXECUTE			0x18
#define CPTRA_MCI_MBOX_TARGET_STATUS		0x1c
#define CPTRA_MCI_MBOX_CMD_STATUS		0x20
#define   CPTRA_MCI_MBOX_CMD_STATUS_PS		GENMASK(3, 0)
#define CPTRA_MCI_MBOX_HW_STATUS		0x24

/* Actual backing size of the mailbox data SRAM */
#define CPTRA_MCI_MBOX_SRAM_SIZE		0x4000

/* Max time to wait for the mailbox LOCK to become available */
#define CPTRA_MCI_MBOX_LOCK_TIMEOUT_US		(1000 * USEC_PER_MSEC)

/*
 * Max time to wait for a mailbox command to complete. Not present in the
 * u-boot source this was ported from (its busy-wait loop never times out) --
 * added because an unbounded poll loop is not acceptable inside the kernel.
 * ML-DSA-87 verification is the slowest command issued through this
 * mailbox, so the bound is generous.
 */
#define CPTRA_MCI_MBOX_EXEC_TIMEOUT_US		(5000 * USEC_PER_MSEC)

/* mbox_status_e (mcu_mbox0_csr.mbox_cmd_status) */
enum cptra_mci_mbox_sts {
	CPTRA_MCI_MBSTS_CMD_BUSY = 0,
	CPTRA_MCI_MBSTS_DATA_READY,
	CPTRA_MCI_MBSTS_CMD_COMPLETE,
	CPTRA_MCI_MBSTS_CMD_FAILURE,
};

/*
 * Direct (non-mailbox-protocol) MCI subsystem status registers. These are
 * plain memory-mapped registers reachable through the same paged SCU1
 * window as the mailbox CSR/SRAM (see SCU1_CPTRA_SS_AXI_WIN in
 * aspeed-cptra-mci.c), each block on its own 64KB-aligned page --
 * confirmed on real hardware via the same address>>16 page encoding as
 * CPTRA_MCI_MBOX_CSR_PAGE/SRAM_PAGE. Unlike the mailbox, there is no
 * command/lock/execute protocol here: aspeed_cptra_mci_reg_read() just
 * selects the page and does a plain register read.
 */

/* mci_reg block, absolute base 0x21000000 */
#define CPTRA_MCI_REG_PAGE			0x2100	/* 0x21000000 >> 16 */

#define CPTRA_MCI_REG_MCU_IFU_AXI_USER			0x0020
#define CPTRA_MCI_REG_MCU_LSU_AXI_USER			0x0024
#define CPTRA_MCI_REG_MCU_SRAM_CONFIG_AXI_USER		0x0028
#define CPTRA_MCI_REG_MCI_SOC_CONFIG_AXI_USER		0x002c
#define CPTRA_MCI_REG_RESET_REASON			0x0038
#define   CPTRA_MCI_REG_RESET_REASON_FW_HITLESS_UPD_RESET	BIT(0)
#define   CPTRA_MCI_REG_RESET_REASON_FW_BOOT_UPD_RESET		BIT(1)
#define   CPTRA_MCI_REG_RESET_REASON_WARM_RESET		BIT(2)
#define CPTRA_MCI_REG_SECURITY_STATE			0x0040
#define   CPTRA_MCI_REG_SECURITY_STATE_DEVICE_LIFECYCLE	GENMASK(1, 0)
#define   CPTRA_MCI_REG_SECURITY_STATE_DEBUG_LOCKED		BIT(2)
#define   CPTRA_MCI_REG_SECURITY_STATE_SCAN_MODE		BIT(3)

/* device_lifecycle_e */
enum cptra_mci_device_lifecycle {
	CPTRA_MCI_DEVICE_UNPROVISIONED = 0,
	CPTRA_MCI_DEVICE_MANUFACTURING = 1,
	CPTRA_MCI_DEVICE_PRODUCTION = 3,
};

/* Each MBOXn_*_AXI_USER block is CPTRA_MCI_REG_MBOX_AXI_USER_COUNT 32-bit regs */
#define CPTRA_MCI_REG_MBOX_AXI_USER_COUNT		5
#define CPTRA_MCI_REG_MBOX0_VALID_AXI_USER(n)		(0x0180 + 4 * (n))
#define CPTRA_MCI_REG_MBOX0_AXI_USER_LOCK(n)		(0x01a0 + 4 * (n))
#define CPTRA_MCI_REG_MBOX1_VALID_AXI_USER(n)		(0x01c0 + 4 * (n))
#define CPTRA_MCI_REG_MBOX1_AXI_USER_LOCK(n)		(0x01e0 + 4 * (n))

#define CPTRA_MCI_REG_SS_DEBUG_INTENT			0x0418
#define CPTRA_MCI_REG_SS_CONFIG_DONE_STICKY		0x0440
#define CPTRA_MCI_REG_SS_CONFIG_DONE			0x0444

/*
 * soc_ifc_reg block, absolute base 0xa0030000 (page base and block base are
 * the same address here). Bit fields for CPTRA_RESET_REASON/
 * CPTRA_SECURITY_STATE cross-checked against
 * caliptra-mcu-sw/registers/generated-firmware/src/soc.rs (CptraResetReason/
 * CptraSecurityState) -- note CPTRA_RESET_REASON only has 2 bits (no
 * hitless/boot split), unlike mci_reg's own 3-bit RESET_REASON.
 */
#define CPTRA_MCI_SOC_IFC_PAGE				0xa003	/* 0xa0030000 >> 16 */

#define CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON		0x0040
#define   CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON_FW_UPD_RESET	BIT(0)
#define   CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON_WARM_RESET	BIT(1)
#define CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE		0x0044
#define   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_DEVICE_LIFECYCLE	GENMASK(1, 0)
#define   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_DEBUG_LOCKED		BIT(2)
#define   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_SCAN_MODE		BIT(3)
#define CPTRA_MCI_SOC_IFC_MBOX_AXI_USER_COUNT		5
#define CPTRA_MCI_SOC_IFC_CPTRA_MBOX_VALID_AXI_USER(n)	(0x0048 + 4 * (n))
#define CPTRA_MCI_SOC_IFC_CPTRA_MBOX_AXI_USER_LOCK(n)	(0x005c + 4 * (n))
#define CPTRA_MCI_SOC_IFC_CPTRA_TRNG_VALID_AXI_USER	0x0070
#define CPTRA_MCI_SOC_IFC_CPTRA_TRNG_AXI_USER_LOCK	0x0074
#define CPTRA_MCI_SOC_IFC_CPTRA_FUSE_VALID_AXI_USER	0x0108

/* aspeed-cptra-mci.c */
int aspeed_cptra_mci_mbox_execute(u32 cmd, const void *req, u32 req_len,
				  void *resp, u32 resp_buf_len, u32 *resp_len);
int aspeed_cptra_mci_mbox_execute_sg(u32 cmd, const void *hdr, u32 hdr_len,
				     const void *data, u32 data_len,
				     void *resp, u32 resp_buf_len, u32 *resp_len);
int aspeed_cptra_mci_reg_read(u32 page, u32 offset, u32 *value);

#endif /* _ASPEED_CPTRA_MCI_H */
