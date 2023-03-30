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
 * @file libved_ve3.h
 * @brief VE3 specific part of VE driver library header
 */
#if !defined(__LIBVED_H)
#error "Never use <libved_ve3.h> directly; include <libved.h> instead."
#endif

#if !defined(__LIBVED_VE3_H)
#define __LIBVED_VE3_H

#if defined(__cplusplus)
extern "C" {
#endif

int vedl_ve3_update_atb_all(vedl_sys_reg_handle *, const ve3_atb_reg_t *);
int vedl_ve3_update_atb_dir(vedl_sys_reg_handle *, const ve3_atb_reg_t *,
				int);
int vedl_ve3_update_atb_entry(vedl_sys_reg_handle *, const ve3_atb_entry_t *,
				int, int);

int vedl_ve3_get_dmaatb_all(const vedl_common_reg_handle *,
				ve3_dmaatb_reg_t *);
int vedl_ve3_update_dmaatb_all(vedl_common_reg_handle *reg,
				const ve3_dmaatb_reg_t *);
int vedl_ve3_update_dmaatb_dir(vedl_common_reg_handle *,
				const ve3_dmaatb_reg_t *, int);
int vedl_ve3_update_dmaatb_entry(vedl_common_reg_handle *,
				const ve3_dmaatb_entry_t *, int, int);
int vedl_ve3_get_pciatb_all(const vedl_common_reg_handle *,
				ve3_pciatb_entry_t *);
int vedl_ve3_update_pciatb_all(vedl_common_reg_handle *,
				const ve3_pciatb_entry_t *);
int vedl_ve3_update_pciatb_entry(vedl_common_reg_handle *,
				const ve3_pciatb_entry_t *, int);

#define vedl_ve3_get_pci_bar0_size(handle, addr) \
	vedl_get_pci_bar_size(handle, addr, 0)
#define vedl_ve3_get_pci_bar0_address(handle, addr) \
	vedl_get_pci_bar_address(handle, addr, 0)

int vedl_ve3_get_pci_bar4_crarea_address(vedl_handle *, uint64_t *);

int vedl_ve3_notify_fault(vedl_handle *handle);
int vedl_ve3_request_ownership(vedl_handle *handle, int timeout);
int vedl_ve3_release_ownership(vedl_handle *handle);
int vedl_ve3_complete_memclear(vedl_handle *handle);
int vedl_ve3_handle_clock_gating(vedl_handle *handle, int flags);
int vedl_get_clock_gating_state(vedl_handle *handle, uint64_t *state)  ;
#if defined(__cplusplus)
}
#endif
#endif				/* if !defined(__LIBVED_VE3_H) */

