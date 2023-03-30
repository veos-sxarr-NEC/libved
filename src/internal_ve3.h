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

#if !defined(__LIBVED_INTERNAL_VE3_H)
#define __LIBVED_INTERNAL_VE3_H

extern const struct vedl_arch_class _vedl_ve3_arch_class;
#define IS_VE3(hdl) \
	(hdl->arch_class == &_vedl_ve3_arch_class)

/* register_ve3.c */
int _vedl_ve3_get_user_regs_area(int, int, struct reg_area *);
int _vedl_ve3_get_sys_regs_area(int, int, struct reg_area *);
int _vedl_ve3_get_common_regs_area(int, struct reg_area *);

int _vedl_ve3_get_usr_offset(ve_usr_reg_name_t,  int *, off_t *);
int _vedl_ve3_get_sys_offset(ve_sys_reg_name_t,  int *, off_t *);

#endif	/* if !defined(__LIBVED_INTERNAL_VE3_H) */
