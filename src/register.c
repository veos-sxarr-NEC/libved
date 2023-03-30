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
 * @file  register.c
 * @brief VE Driver library architecture-independent register access functions
 */

#define _GNU_SOURCE
#include <errno.h>
#include "libved.h"
#include "internal.h"
#include "register.h"

/**
 * \addtogroup VEDL API
 */
//@{
/**
 * @brief get the size of user register area
 *
 * @param[in] reg core user register handle
 * @param area_id user register area ID
 *
 * @return size of user register area; zero upon failure.
 */
size_t vedl_get_usr_reg_size(const vedl_user_reg_handle *reg, int area_id)
{
	if (area_id < 0 || reg->user_regs.num_areas <= area_id)
		return 0;
	return reg->user_regs.areas[area_id].size;
}

/**
 * @brief get user register
 *
 * @param[in] from core user register handle
 * @param area_id user register area ID
 * @param offset offset from top of core user register
 * @param[out] to register value will be stored here
 * @param size register size to get
 *
 * @return 0 on success. negative on failure.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_get_usr_reg_words(const vedl_user_reg_handle *from, int area_id,
		off_t offset, void *to, size_t size)
{
	return _read_reg(from->user_regs.areas[area_id].start, offset,
				to, size);
}

/**
 * @brief get specified user register (8 Byte register)
 *
 * @param[in] from core user register handle
 * @param reg register name
 * @param[out] regdata buffer to read
 *
 * @return 0 on success. negative on failure.
 *         -3 when specified register is invalid
 */
int vedl_get_usr_reg(const vedl_user_reg_handle *from,
		     ve_usr_reg_name_t reg, ve_reg_t *regdata)
{
	off_t offset;
	int area_id;
	int rv = from->get_usr_offset(reg, &area_id, &offset);

	if (rv < 0)
		return -3;

	return _read_reg_word(from->user_regs.areas[area_id].start, offset,
				regdata);
}

/**
 * @brief get all user register
 *
 * @param[in] from core user register handle
 * @param area_id core user register area ID
 * @param[out] to buffer to read register data
 * @param size of buffer
 *
 * @return 0 on success. negative on failure.
 */
int vedl_get_usr_reg_all(const vedl_user_reg_handle *from, int area_id,
			 void *to, size_t size)
{
	size_t reg_size = vedl_get_usr_reg_size(from, area_id);
	if (reg_size == 0 || size != reg_size)
		return -EINVAL;

	return vedl_get_usr_reg_words(from, area_id, 0, to, reg_size);
}

/**
 * @brief set user register
 *
 * @param[out] to core user register handle
 * @param area_id core user register area ID
 * @param offset offset of register from top of core user register
 * @param[in] from value to set
 * @param size size to set
 *
 * @return 0 on success. negative on failure.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_set_usr_reg_words(vedl_user_reg_handle *to, int area_id,
		off_t offset, const void *from, size_t size)
{
	return _write_reg(to->user_regs.areas[area_id].start, offset,
				from, size);
}

/**
 * @brief set specified core user register (8 Byte register)
 *
 * @param[in] to core user register handle
 * @param reg register name
 * @param regdata register data to store
 *
 * @return 0 on success. negative on failure.
 *         -3 when specified register is invalid
 */
int vedl_set_usr_reg(vedl_user_reg_handle *to, ve_usr_reg_name_t reg,
			ve_reg_t regdata)
{
	off_t offset;
	int area_id;
	int rv = to->get_usr_offset(reg, &area_id, &offset);

	if (rv < 0)
		return -3;

	return _write_reg_word(to->user_regs.areas[area_id].start, offset,
				&regdata);
}

/**
 * @brief set all core user register
 *
 * @param[in] to core user register handle
 * @param area_id core user register area ID
 * @param[in] from buffer to write data
 * @param size size of buffer
 *
 * @return 0 on success. negative on failure.
 */
int vedl_set_usr_reg_all(vedl_user_reg_handle *to, int area_id,
			 const void *from, size_t size)
{
	size_t reg_size = vedl_get_usr_reg_size(to, area_id);
	if (reg_size == 0 || size != reg_size)
		return -EINVAL;

	return vedl_set_usr_reg_words(to, area_id, 0, from, reg_size);
}

/**
 * @brief get the size of system register area
 *
 * @param[in] reg core system register handle
 * @param area_id system register area ID
 *
 * @return size of user register area; zero upon failure.
 */
size_t vedl_get_sys_reg_size(const vedl_sys_reg_handle *reg, int area_id)
{
	if (area_id < 0 || reg->sys_regs.num_areas <= area_id)
		return 0;
	return reg->sys_regs.areas[area_id].size;
}

/**
 * @brief get system register
 *
 * @param[in] from core system register handle
 * @param area_id system register area ID
 * @param offset offset from top of core system register
 * @param[out] to register value will be stored here
 * @param size register size to get
 *
 * @return 0 on success. negative on failure.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_get_sys_reg_words(const vedl_sys_reg_handle *from, int area_id,
		off_t offset, void *to, size_t size)
{
	return _read_reg(from->sys_regs.areas[area_id].start, offset,
				to, size);
}

/**
 * @brief get specified system register (8 Byte register)
 *
 * @param[in] from core system register handle
 * @param reg register name
 * @param[out] regdata buffer to read
 *
 * @return 0 on success. negative on failure.
 *         -3 when specified register is invalid
 */
int vedl_get_sys_reg(const vedl_sys_reg_handle *from,
		     ve_sys_reg_name_t reg, ve_reg_t *regdata)
{
	off_t offset;
	int area_id;
	int rv = from->get_sys_offset(reg, &area_id, &offset);

	if (rv < 0)
		return -3;

	return _read_reg_word(from->sys_regs.areas[area_id].start, offset,
				regdata);
}

/**
 * @brief get all system register
 *
 * @param[in] from core system register handle
 * @param area_id core system register area ID
 * @param[out] to buffer to read register data
 * @param size of buffer
 *
 * @return 0 on success. negative on failure.
 */
int vedl_get_sys_reg_all(const vedl_sys_reg_handle *from, int area_id,
			 void *to, size_t size)
{
	size_t reg_size = vedl_get_sys_reg_size(from, area_id);
	if (reg_size == 0 || size != reg_size)
		return -EINVAL;

	return vedl_get_sys_reg_words(from, area_id, 0, to, reg_size);
}

/**
 * @brief set system register
 *
 * @param[out] to core system register handle
 * @param area_id core system register area ID
 * @param offset offset of register from top of core system register
 * @param[in] from value to set
 * @param size size to set
 *
 * @return 0 on success. negative on failure.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_set_sys_reg_words(vedl_sys_reg_handle *to, int area_id,
		off_t offset, const void *from, size_t size)
{
	return _write_reg(to->sys_regs.areas[area_id].start, offset,
				from, size);
}

/**
 * @brief set specified system user register (8 Byte register)
 *
 * @param[in] to core system register handle
 * @param reg register name
 * @param regdata register data to store
 *
 * @return 0 on success. negative on failure.
 *         -3 when specified register is invalid
 */
int vedl_set_sys_reg(vedl_sys_reg_handle *to, ve_sys_reg_name_t reg,
			ve_reg_t regdata)
{
	off_t offset;
	int area_id;
	int rv = to->get_sys_offset(reg, &area_id, &offset);

	if (rv < 0)
		return -3;

	return _write_reg_word(to->sys_regs.areas[area_id].start, offset,
				&regdata);
}

/**
 * @brief set all core system register
 *
 * @param[in] to core system register handle
 * @param area_id core system register area ID
 * @param[in] from buffer to write data
 * @param size size of buffer
 *
 * @return 0 on success. negative on failure.
 */
int vedl_set_sys_reg_all(vedl_sys_reg_handle *to, int area_id,
			 const void *from, size_t size)
{
	size_t reg_size = vedl_get_sys_reg_size(to, area_id);
	if (reg_size == 0 || size != reg_size)
		return -EINVAL;

	return vedl_set_sys_reg_words(to, area_id, 0, from, reg_size);
}

/**
 * @brief get common register
 *
 * @param[in] from system common register handle
 * @param area_id system common register area ID
 * @param offset offset from top of system common register
 * @param[out] to buffer to read
 * @param size size of buffer
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_get_cnt_reg_words(const vedl_common_reg_handle *from, int area_id,
		off_t offset, void *to, size_t size)
{
	return _read_reg(from->common_regs.areas[area_id].start, offset,
				to, size);
}

/**
 * @brief get common register (8byte)
 *
 * @param[in] from system common register handle
 * @param area_id system common register area ID
 * @param offset offset from top of system common register
 * @param[out] regdata buffer to read
 *
 * @return 0 on success. negative on failure.
 *         -2 on invalid offset
 */
int vedl_get_cnt_reg_word(const vedl_common_reg_handle *from, int area_id,
				off_t offset, ve_reg_t *regdata)
{
	return _read_reg_word(from->common_regs.areas[area_id].start, offset,
			regdata);
}

/**
 * @brief set common register
 *
 * @param[out] to system common register handle
 * @param area_id system common register area ID
 * @param offset offset from top of system common register
 * @param[in] from buffer to write
 * @param size size of buffer
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
int vedl_set_cnt_reg_words(vedl_common_reg_handle *to, int area_id,
		off_t offset, const void *from, size_t size)
{
	return _write_reg(to->common_regs.areas[area_id].start, offset,
				from, size);
}

/**
 * @brief set common register (8byte)
 *
 * @param[out] to system common register handle
 * @param area_id system common register area ID
 * @param offset offset from top of system common register
 * @param regdata register data to store
 *
 * @return 0 on success. negative on failure.
 *         -2 on invalid offset
 */
int vedl_set_cnt_reg_word(vedl_common_reg_handle *to, int area_id,
		off_t offset, ve_reg_t regdata)
{
	return _write_reg_word(to->common_regs.areas[area_id].start, offset,
				&regdata);
}
//@}
