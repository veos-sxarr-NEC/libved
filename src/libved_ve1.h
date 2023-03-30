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
 * @file libved_ve1.h
 * @brief VE1 specific part of VE driver library header
 */
#if !defined(__LIBVED_H)
#error "Never use <libved_ve1.h> directly; include <libved.h> instead."
#endif

#if !defined(__LIBVED_VE1_H)
#define __LIBVED_VE1_H

#if defined(__cplusplus)
extern "C" {
#endif

int vedl_ve1_update_atb_all(vedl_sys_reg_handle *, const ve1_atb_reg_t *);
int vedl_ve1_update_atb_dir(vedl_sys_reg_handle *, const ve1_atb_reg_t *,
				int);
int vedl_ve1_update_atb_entry(vedl_sys_reg_handle *, const ve1_atb_entry_t *,
				int , int);

int vedl_ve1_get_dmaatb_all(const vedl_common_reg_handle *,
				ve1_dmaatb_reg_t *);
int vedl_ve1_update_dmaatb_all(vedl_common_reg_handle *,
				const ve1_dmaatb_reg_t *);
int vedl_ve1_update_dmaatb_dir(vedl_common_reg_handle *,
				const ve1_dmaatb_reg_t *, int);
int vedl_ve1_update_dmaatb_entry(vedl_common_reg_handle *,
				const ve1_atb_entry_t *, int, int);
int vedl_ve1_get_pciatb_all(const vedl_common_reg_handle *,
				ve1_pciatb_entry_t *);
int vedl_ve1_update_pciatb_all(vedl_common_reg_handle *,
				const ve1_pciatb_entry_t *);
int vedl_ve1_update_pciatb_entry(vedl_common_reg_handle *,
				const ve1_pciatb_entry_t *, int);

#define vedl_ve1_get_pci_bar0_size(handle, addr) \
	vedl_get_pci_bar_size(handle, addr, 0)
#define vedl_ve1_get_pci_bar0_address(handle, addr) \
	vedl_get_pci_bar_address(handle, addr, 0)

#define vedl_ve1_get_crarea_address(handle, addr) \
	vedl_get_pci_bar_address(handle, addr, 3)

int vedl_ve1_update_firmware(vedl_handle *handle);
int vedl_update_firmware(vedl_handle *handle);

#if defined(__cplusplus)
}
#endif
#endif				/* if !defined(__LIBVED_VE1_H) */

