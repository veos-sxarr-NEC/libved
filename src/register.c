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
 * @brief VE Driver library register access functions
 */

#define _GNU_SOURCE
#include "ve_drv.h"
#include "vp.h"
#include "libved.h"
#include "internal.h"

/**
 * @brief get specific user register (8 Byte register)
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of mapped address of core user register
 * @param reg specific register
 * @param[out] regdata readed data will be stored here
 *
 * @return 0 on success. negative on failure.
 *         -3 when specified register is invalid
 */
int vedl_get_usr_reg(vedl_handle *handle, core_user_reg_t *addr,
		     usr_reg_name_t reg, reg_t *regdata)
{
	off_t offset;

	offset = _get_reg_offset(USR_REG, (int)reg);
	if (offset < 0)
		return -3;

	return _read_reg_word(handle, addr, offset, regdata,
			sizeof(reg_t));
}

/**
 * @brief get all user register
 *
 * @param[in] handle VEDL handle
 * @param[in] from head of mapped address of core user register
 * @param[out] to readed data will be stored here.
 *
 * @return 0 on success. negative on failure.
 */
int vedl_get_usr_reg_all(vedl_handle *handle, core_user_reg_t *from,
			 core_user_reg_t *to)
{
	size_t reg_size = offsetof(core_user_reg_t, VR) + sizeof(from->VR);

	return vedl_get_usr_reg_words(handle, from, 0, to, reg_size);
}

/**
 * @brief set specific core user register (8 Byte register)
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of mapped address of core user register
 * @param reg specific register
 * @param regdata register data to store
 *
 * @return 0 on success. negative on failure.
 *         -3 when specified register is invalid
 */
int vedl_set_usr_reg(vedl_handle *handle, core_user_reg_t *addr,
		     usr_reg_name_t reg, reg_t regdata)
{
	off_t offset;

	offset = _get_reg_offset(USR_REG, (int)reg);
	if (offset < 0)
		return -3;

	return _write_reg_word(handle, addr, offset, &regdata,
			sizeof(reg_t));
}

/**
 * @brief set all core user register
 *
 * @param[in] handle VEDL handler
 * @param[in] to head of mapped address of core user register
 * @param[in] from head of core_user_reg_t
 *
 * @return 0 on success. negative on failure.
 */
int vedl_set_usr_reg_all(vedl_handle *handle, core_user_reg_t *to,
			 core_user_reg_t *from)
{
	size_t reg_size = offsetof(core_user_reg_t, VR) + sizeof(from->VR);

	return vedl_set_usr_reg_words(handle, to, 0, from, reg_size);
}

/**
 * @brief get specific system register (8 Byte register)
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of mapped address of core system register
 * @param reg specific register
 * @param[out] regdata readed data will be stored here
 *
 * @return 0 on success. negative on failure.
 *         -3 when specified register is invalid
 */
int vedl_get_sys_reg(vedl_handle *handle, core_system_reg_t *addr,
		     sys_reg_name_t reg, reg_t *regdata)
{
	off_t offset;

	offset = _get_reg_offset(SYS_REG, (int)reg);
	if (offset < 0)
		return -3;

	return _read_reg_word(handle, addr, offset, regdata,
			sizeof(reg_t));
}

/**
 * @brief get all system register
 *
 * @param[in] handle VEDL handle
 * @param[in] from head of mapped address of core system register
 * @param[out] to readed data will be stored here.
 *
 * @return 0 on success. negative on failure.
 */
int vedl_get_sys_reg_all(vedl_handle *handle, core_system_reg_t *from,
			 core_system_reg_t *to)
{
	size_t reg_size = offsetof(core_system_reg_t, DIDR) +
		sizeof(from->DIDR);

	return vedl_get_sys_reg_words(handle, from, 0, to, reg_size);
}

/**
 * @brief set specific system register (8 Byte register)
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of mapped address of core system register
 * @param reg specific register
 * @param[in] regdata register data to store
 *
 * @return 0 on success. negative on failure.
 *         -3 on invalid reg
 */
int vedl_set_sys_reg(vedl_handle *handle, core_system_reg_t *addr,
		     sys_reg_name_t reg, reg_t regdata)
{
	off_t offset;

	offset = _get_reg_offset(SYS_REG, (int)reg);
	if (offset < 0)
		return -3;

	return _write_reg_word(handle, addr, offset, &regdata,
			sizeof(reg_t));
}

/**
 * @brief set all system register
 *
 * @param[in] handle VEDL handle
 * @param[in] to head of mapped address of core system register
 * @param[in] from head of core_system_reg_t which contains
 *            register data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_set_sys_reg_all(vedl_handle *handle, core_system_reg_t *to,
			 core_system_reg_t *from)
{
	int ret;
	size_t size;
	off_t exsrar_off = offsetof(core_system_reg_t, EXSRAR);
	size_t exsrar_size = sizeof(from->EXSRAR);
	size_t reg_size = offsetof(core_system_reg_t, DIDR) +
		sizeof(from->DIDR);

	size = (size_t)(exsrar_off - 0);
	ret = vedl_set_sys_reg_words(handle, to, 0, from, size);
	if (ret)
		return ret;

	/* skip EXSRAR (EXSRAR is set by VE Driver) */

	to = (void *)to + exsrar_off + exsrar_size;
	from = (void *)from + exsrar_off + exsrar_size;
	size = (size_t)(reg_size - (exsrar_off + exsrar_size));
	ret = vedl_set_sys_reg_words(handle, to, 0, from, size);

	return ret;
}

/**
 * @brief update whole ATB (entire space)
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of core_system_reg_t which contains ATB to update
 * @param[in] atb ATB data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_update_atb_all(vedl_handle *handle, core_system_reg_t *addr,
			atb_reg_t *atb)
{
	return vedl_set_sys_reg_words(handle, addr,
			offsetof(core_system_reg_t, atb), atb,
			sizeof(atb_reg_t));
}

/**
 * @brief update 1 ATB Space
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of core_system_reg_t which contains ATB to update
 * @param[in] atb ATB data to store
 * @param dirnum ATB directory number
 *
 * @return 0 on success. negative on failure.
 *         -3 when dirnum is invalid
 */
int vedl_update_atb_dir(vedl_handle *handle, core_system_reg_t *addr,
			atb_reg_t *atb, int dirnum)
{
	int ret;
	off_t atb_offset;
	off_t atb_dir_offset;
	off_t atb_dir_entry_offset;

	if (dirnum >= ATB_DIR_NUM)
		return -3;

	atb_offset = offsetof(core_system_reg_t, atb);
	atb_dir_offset = atb_offset + offsetof(atb_reg_t, dir[dirnum]);
	atb_dir_entry_offset = atb_offset + offsetof(atb_reg_t, entry[dirnum]);

	/* update atb_dir_t */
	ret = vedl_set_sys_reg_words(handle, addr, atb_dir_offset,
			&atb->dir[dirnum], sizeof(atb_dir_t));
	if (ret)
		return ret;

	/* update atb_entry_t */
	ret = vedl_set_sys_reg_words(handle, addr, atb_dir_entry_offset,
		   atb->entry[dirnum],
		   sizeof(atb_entry_t) * ATB_ENTRY_MAX_SIZE);

	return ret;
}

/**
 * @brief update 1 ATB Entry
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of core_system_reg_t which contains ATB to update
 * @param[in] entry ATB entry data to store
 * @param dirnum ATB directory number
 * @param entnum ATB entry number
 *
 * @return 0 on success. negative on failure.
 *         -3 when dirnum is invalid
 *         -4 when entnum is invalid
 */
int vedl_update_atb_entry(vedl_handle *handle, core_system_reg_t *addr,
			  atb_entry_t *entry, int dirnum, int entnum)
{
	off_t atb_offset;
	off_t atb_entry_offset;

	if (dirnum >= ATB_DIR_NUM)
		return -3;
	if (entnum >= ATB_ENTRY_MAX_SIZE)
		return -4;

	atb_offset = offsetof(core_system_reg_t, atb);
	atb_entry_offset =
	    atb_offset + offsetof(atb_reg_t, entry[dirnum][entnum]);

	/* update 1 entry */
	return _write_reg_word(handle, addr, atb_entry_offset, entry,
		   sizeof(atb_entry_t));
}

/**
 * @brief get common register (8byte)
 *
 * @param[in] handle VEDL handle
 * @param[in] from head of mapped address of system common register
 * @param offset offset from top of system common register
 * @param[out] to register value will be sotred
 * @param size size to get
 *
 * @return 0 on success. negative on failure.
 *         -2 on invalid offset
 */
int vedl_set_cnt_reg_word(vedl_handle *handle, system_common_reg_t *to,
		off_t offset, reg_t val)
{
	return _write_reg_word(handle, to, offset, &val, sizeof(val));
}

/**
 * @brief set common register (8byte)
 *
 * @param[in] handle VEDL handle
 * @param[out] to head of mapped address of node control register
 * @param offset offset from top of system common register
 * @param[in] from the value to be stored
 * @param size size to set
 *
 * @return 0 on success. negative on failure.
 *         -2 on invalid offset
 */
int vedl_get_cnt_reg_word(vedl_handle *handle, system_common_reg_t *from,
		off_t offset, reg_t *valp)
{
	return _read_reg_word(handle, from, offset, valp,
			sizeof(valp));
}

/**
 * @brief get whole DMAATB
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of system_common_reg_t
 * @param[out] dmaatb buffer for storing DMAATB data
 *
 * @return 0 on success. negative on failure.
 */
int vedl_get_dmaatb_all(vedl_handle *handle, system_common_reg_t *addr,
			dmaatb_reg_t *dmaatb)
{
	return vedl_get_cnt_reg_words(handle, addr,
			offsetof(system_common_reg_t, dmaatb),
			dmaatb, sizeof(dmaatb_reg_t));
}

/**
 * @brief update whole DMAATB
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of system_common_reg_t which contains DMAATB to update
 * @param[in] dmaatb DMAATB data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_update_dmaatb_all(vedl_handle *handle, system_common_reg_t *addr,
			   dmaatb_reg_t *dmaatb)
{
	return vedl_set_cnt_reg_words(handle, addr,
			offsetof(system_common_reg_t, dmaatb),
			dmaatb, sizeof(dmaatb_reg_t));
}

/**
 * @brief update 1 DMAATB Space
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of system_common_reg_t which contains DMAATB to update
 * @param[in] dmaatb DMAATB data to store
 * @param dirnum DMAATB directory number
 *
 * @return 0 on success. negative on failure.
 *         -3 when dirnum is invalid
 */
int vedl_update_dmaatb_dir(vedl_handle *handle, system_common_reg_t *addr,
			   dmaatb_reg_t *dmaatb, int dirnum)
{
	int ret;
	off_t atb_offset;
	off_t atb_dir_offset;
	off_t atb_dir_entry_offset;

	if (dirnum >= DMAATB_DIR_NUM)
		return -3;

	atb_offset = offsetof(system_common_reg_t, dmaatb);
	atb_dir_offset = atb_offset + offsetof(dmaatb_reg_t, dir[dirnum]);
	atb_dir_entry_offset = atb_offset + offsetof(dmaatb_reg_t,
						     entry[dirnum]);

	/* update atb_dir_t */
	ret = vedl_set_cnt_reg_words(handle, addr, atb_dir_offset,
			&dmaatb->dir[dirnum], sizeof(atb_dir_t));
	if (ret)
		return ret;

	/* update atb_entry_t */
	ret = vedl_set_cnt_reg_words(handle, addr, atb_dir_entry_offset,
		   dmaatb->entry[dirnum],
		   sizeof(atb_entry_t) * DMAATB_ENTRY_MAX_SIZE);

	return ret;
}

/**
 * @brief update 1 DMAATB Entry
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of system_common_reg_t which contains DMAATB to update
 * @param[in] entry DMAATB entry data to store
 * @param dirnum DMAATB directory number
 * @param entnum DMAATB entry number
 *
 * @return 0 on success. negative on failure.
 *         -3 when dirnum is invalid
 *         -4 when entnum is invalid
 */
int vedl_update_dmaatb_entry(vedl_handle *handle, system_common_reg_t *addr,
			     atb_entry_t *entry, int dirnum, int entnum)
{
	off_t atb_offset;
	off_t atb_entry_offset;

	if (dirnum >= DMAATB_DIR_NUM)
		return -3;
	if (entnum >= DMAATB_ENTRY_MAX_SIZE)
		return -4;

	atb_offset = offsetof(system_common_reg_t, dmaatb);
	atb_entry_offset = atb_offset + offsetof(dmaatb_reg_t,
						 entry[dirnum][entnum]);

	/* update 1 entry */
	return _write_reg_word(handle, addr, atb_entry_offset, entry,
		   sizeof(atb_entry_t));
}

/**
 * @brief get whole PCIATB
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of system_common_reg_t
 * @param[in] pciatb buffer for storing PCIATB
 *
 * @return 0 on success. negative on failure.
 */
int vedl_get_pciatb_all(vedl_handle *handle, system_common_reg_t *addr,
			pciatb_entry_t *pciatb)
{
	return vedl_get_cnt_reg_words(handle, addr,
			offsetof(system_common_reg_t, pciatb), pciatb,
			sizeof(pciatb_entry_t) * PCIATB_VLD_SIZE);
}

/**
 * @brief update whole PCIATB
 *
 * @param[in] handle VEDL handle
 * @param[in] addr head of system_common_reg_t which contains PCIATB to update
 * @param[in] pciatb PCIATB data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_update_pciatb_all(vedl_handle *handle, system_common_reg_t *addr,
			   pciatb_entry_t *pciatb)
{
	return vedl_set_cnt_reg_words(handle, addr,
			offsetof(system_common_reg_t, pciatb), pciatb,
			sizeof(pciatb_entry_t) * PCIATB_VLD_SIZE);
}

/**
 * @brief update 1 PCIATB Entry
 *
 * @param[in] handle
 * @param[in] addr head of system_common_reg_t which contains PCIATB to update
 * @param[in] entry PCIATB entry data to store
 * @param entnum PCIATB entry number
 *
 * @return 0 on success. negative on failure.
 *         -3 when entnum is invalid
 */
int vedl_update_pciatb_entry(vedl_handle *handle, system_common_reg_t *addr,
			     pciatb_entry_t *entry, int entnum)
{
	off_t atb_entry_offset;

	if (entnum >= PCIATB_EXT_SIZE)
		return -3;

	atb_entry_offset = offsetof(system_common_reg_t, pciatb[entnum]);

	/* update 1 entry */
	return _write_reg_word(handle, addr, atb_entry_offset, entry,
		   sizeof(atb_entry_t));
}
