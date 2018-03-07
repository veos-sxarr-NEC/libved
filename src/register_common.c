/*
 * VE Driver Library
 *
 * Copyright (C) 2017-2018 NEC Corporation
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
 * @file  register_common.c
 * @brief VE Driver library register access common functions
 */

#define _GNU_SOURCE
#include "ve_drv.h"
#include "vp.h"
#include "libved.h"
#include "internal.h"

/**
 * @brief get user register
 *
 * @param[in] handle VEDL handle
 * @param[in] from head of mapped address of core user register
 * @param offset offset from top of core user register
 * @param[out] to register value will be stored here
 * @param size register size to get
 *
 * @return 0 on success. negative on failure.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_get_usr_reg_words(vedl_handle *handle, core_user_reg_t *from,
		off_t offset, void *to, size_t size)
{
	return _read_reg(handle, from, offset, to, size);
}

/**
 * @brief set user register
 *
 * @param[in] handle VEDL handler
 * @param[out] to head of mapped address of core user register
 * @param offset offset of register from top of core user register
 * @param[in] from value to set
 * @param size size to set
 *
 * @return 0 on success. negative on failure.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_set_usr_reg_words(vedl_handle *handle, core_user_reg_t *to,
		off_t offset, void *from, size_t size)
{
	return _write_reg(handle, to, offset, from, size);
}

/**
 * @brief get system register
 *
 * @param[in] handle VEDL handle
 * @param[in] from head of mapped address of core system register
 * @param offset offset from top of core system register
 * @param[out] to register value would be stored here
 * @param size size to get
 *
 * @return 0 on success. negative on failure.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_get_sys_reg_words(vedl_handle *handle, core_system_reg_t *from,
		off_t offset, void *to, size_t size)
{
	return _read_reg(handle, from, offset, to, size);
}

/**
 * @brief set system register
 *
 * @param[in] handle VEDL handle
 * @param[out] to head of mapped address of core system register
 * @param offset offset from top of core system register
 * @param[in] from register data to set
 * @param size size to set
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_set_sys_reg_words(vedl_handle *handle, core_system_reg_t *to,
		off_t offset, void *from, size_t size)
{
	return _write_reg(handle, to, offset, from, size);
}

/**
 * @brief get common register
 *
 * @param[in] handle VEDL handle
 * @param[in] from head of mapped address of system common register
 * @param offset offset from top of system common register
 * @param[out] to register value will be sotred
 * @param size size to get
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_get_cnt_reg_words(vedl_handle *handle, system_common_reg_t *from,
		off_t offset, void *to, size_t size)
{
	return _read_reg(handle, from, offset, to, size);
}

/**
 * @brief set common register
 *
 * @param[in] handle VEDL handle
 * @param[out] to head of mapped address of node control register
 * @param offset offset from top of system common register
 * @param[in] from the value to be stored
 * @param size size to set
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_set_cnt_reg_words(vedl_handle *handle, system_common_reg_t *to,
		off_t offset, void *from, size_t size)
{
	return _write_reg(handle, to, offset, from, size);
}
