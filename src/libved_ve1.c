 /*
 * VE Driver Library
 *
 * Copyright (C) 2017-2020 NEC Corporation
 * This file is part of the VE Driver Library.
 *
 * The VE Driver Library is free software; you can redistribute it
 * and/or modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either version
 * 2.1 of the License, or (at your option) any later version.
 *
 * The VE Driver Library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with the VE Driver Library; if not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file  libved_ve1.c
 * @brief VE1 specific part of VE driver library
 */

#define _GNU_SOURCE
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#define _VE_ARCH_VE1_ (1)
#include <ve_drv.h>
#include "libved.h"
#include "ve_hw.h"
#include "internal.h"

#define CHECK_VE1(hdl) \
	do { if (!IS_VE1(hdl)) { \
		errno = ENOTSUP; \
		return -1; \
	} } while (0)

static inline int ve1_do_update_firmware(vedl_handle *handle)
{
	return ioctl(handle->vefd, VEDRV_CMD_VE1_UPDATE_FIRMWARE);
}

/**
 * @brief Update VE firmware
 * @details This function will update VE firmware manually.
 *         Usually it's done automatically at the time of driver
 *         initialization.
 *         Please note that AER is disabled during updating.
 *
 * @param[in] handle VEDL handle
 *
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           EAGAIN on invalid VE state
 *           ENOMEM on lack of memory
 *           EINVAL on invalid argument
 *           EIO on failure of firmware update
 */
int vedl_ve1_update_firmware(vedl_handle *handle)
{
	CHECK_VE1(handle);
	return ve1_do_update_firmware(handle);
}


static int ve1_interrupt_entry_to_msix_number(vedl_interrupt_entry_t entry)
{
	if (0 <= entry && entry <= VE_INTR_CORE15)
		return entry;

	if (VE_INTR_CORE00_HDMA <= entry && entry <= VE_INTR_CORE15_EDMA)
		return entry - VE_INTR_CORE00_HDMA + 16;

	if (VE_INTR_CORE00_FAULT <= entry && entry <= VE_INTR_CORE15_FAULT)
		return entry - VE_INTR_CORE00_FAULT + 64;

	switch (entry) {
	case VE_INTR_P_DMA:
		return 48;
	case VE_INTR_PCI_ACCESS_EXC:
		return 56;
	case VE_INTR_FAULT:
		return 80;
	case VE_INTR_FAULT_DIAG:
		return 81;
	case VE_INTR_FAULT_ENV:
		return 82;
	case VE_INTR_HMC_ERR:
		return 84;
	case VE_INTR_LLC_ERR:
		return 85;
	case VE_INTR_MCU_ERR:
		return 86;
	case VE_INTR_DP_ERR:
		return 87;
	case VE_INTR_VR_ERR:
		return 88;
	case VE_INTR_WARN_ENV:
		return 91;
	case VE_INTR_DIAG_INFO:
		return 92;
	default:
		return -1;
	}
}

static int ve1_set_wait_irq(struct ve1_wait_irq *cond, int msix_entry)
{
	cond->ve_wait_irq_type = VEDRV_IRQ_TYPE_VE1;
	cond->upper = 0;
	cond->lower = 0;
	if (msix_entry < 0 || 128 <= msix_entry)
		return EINVAL;
	if (msix_entry < 64)
		cond->lower = 1UL << msix_entry;
	else
		cond->upper = 1UL << (msix_entry - 64);
	return 0;
}

static int ve1_create_wait_irq(vedl_interrupt_entry_t entry,
				struct ve_wait_irq **irq)
{
	int rv, msix_entry;
	*irq = malloc(sizeof(struct ve1_wait_irq));
	if (*irq == NULL)
		return errno;
	msix_entry = ve1_interrupt_entry_to_msix_number(entry);
	rv = ve1_set_wait_irq((struct ve1_wait_irq *)*irq, msix_entry);
	if (rv != 0) {
		free(*irq);
		*irq = NULL;
	}
	return rv;
}

/* for compatibilities with old libved */
int vedl_ve1_wait_interrupt(vedl_handle *handle, int msix_entry,
			struct timespec *timeout)
{
	struct ve1_wait_irq cond;
	int rv;
	CHECK_VE1(handle);
	rv = ve1_set_wait_irq(&cond, msix_entry);
	if (rv != 0) {
		errno = rv;
		return -1;
	}
	return vedl__wait_for_interrupt(handle, (struct ve_wait_irq *)&cond,
						timeout);
}

const struct vedl_arch_class _vedl_ve1_arch_class = {
	.name = VE_DRV_ARCH_NAME_VE1,
	/* BAR01 (bar0_addr and bar0_size) and BAR3 (bar3_addr and bar3_size) */
	.bars_map = (1 << 0) | (1 << 3),
	.num_user_regs_area = 1,
	.get_user_regs_area = _vedl_ve1_get_user_regs_area,
	.get_usr_offset = _vedl_ve1_get_usr_offset,
	.num_sys_regs_area = 1,
	.get_sys_regs_area = _vedl_ve1_get_sys_regs_area,
	.get_sys_offset = _vedl_ve1_get_sys_offset,
	.num_common_regs_area = 1,
	.get_common_regs_area = _vedl_ve1_get_common_regs_area,
	.create_wait_irq = ve1_create_wait_irq,
	.update_firmware = vedl_ve1_update_firmware,
	.wait_interrupt = vedl_ve1_wait_interrupt,
};
VEDL_ARCH_CLASS(_vedl_ve1_arch_class);
