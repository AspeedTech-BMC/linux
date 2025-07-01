/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef __LINUX_MCTP_PCIE_VDM_H
#define __LINUX_MCTP_PCIE_VDM_H

#include <linux/device.h>
#include <linux/notifier.h>

struct mctp_pcie_vdm_dev;
struct mctp_pcie_vdm_ops {
	int (*send_packet)(struct device *dev, u8 *data, size_t len);
	u8 *(*recv_packet)(struct device *dev);
	void (*free_packet)(void *packet);
	void (*uninit)(struct device *dev);
};

struct mctp_pcie_vdm_dev *mctp_pcie_vdm_add_dev(struct device *dev);
void mctp_pcie_vdm_remove_dev(struct mctp_pcie_vdm_dev *vdm_dev);
void mctp_pcie_vdm_notify_rx(struct mctp_pcie_vdm_dev *vdm_dev);
void mctp_pcie_vdm_register_ops(struct mctp_pcie_vdm_dev *vdm_dev,
				const struct mctp_pcie_vdm_ops *ops);

#endif	 /* __LINUX_MCTP_PCIE_VDM_H */
