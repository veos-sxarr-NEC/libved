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

#if !defined(__LIBVED_INTERNAL_H)
#define __LIBVED_INTERNAL_H

/**
 * @brief Check size and offset of MMIO
 *
 * @param offset offset from top address
 * @param size size to transfer
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
static inline int _check_size_offset(off_t offset, size_t size)
{
	/* size must be multiple of 8byte */
	if (size % 8)
		return -1;
	/* offset must be 8byte align */
	if (offset % 8 || offset < 0)
		return -2;

	return 0;
}

/**
 * @brief read register
 *
 * @param[in] handle VEDL handler
 * @param[in] from_addr register mapped address
 * @param offset offset from from_addr
 * @param[out] to_addr buffer for writing
 * @param size size to read
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
static inline int _read_reg(vedl_handle *handle, void *from_addr,
			     off_t offset, void *to_addr, size_t size)
{
	int err = 0;
#ifdef NO_MEMCPY
	size_t now;
	uint64_t *from8 = (uint64_t *)(from_addr + offset);
	uint64_t *to8 = (uint64_t *)to_addr;
#endif

	err = _check_size_offset(offset, size);
	if (err)
		return err;
	/* read reg */
#ifdef NO_MEMCPY
	for (now = 0; now < size; now += 8)
		*to8++ = *from8++;
#else
	memcpy(to_addr, (void *)(from_addr + offset), size);
#endif

	return 0;
}

/**
 * @brief write register
 *
 * @param[in] handle VEDL handler
 * @param[in] to_addr register mapped address
 * @param offset offset from to_addr
 * @param[in] from_addr buffer for writing
 * @param size size to write
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
static inline int _write_reg(vedl_handle *handle, void *to_addr,
			      off_t offset, void *from_addr, size_t size)
{
	int err = 0;
#ifdef NO_MEMCPY
	size_t now;
	uint64_t *to8 = (uint64_t *)(to_addr + offset);
	uint64_t *from8 = (uint64_t *)from_addr;
#endif

	err = _check_size_offset(offset, size);
	if (err)
		return err;
	/* write reg */
#ifdef NO_MEMCPY
	for(now = 0; now < size; now +=8)
		*to8++ = *from8++;	
#else
	memcpy((void *)(to_addr + offset), from_addr, size);
#endif

	return 0;
}

/**
 * @brief return offset of core_user_reg_t (internal function)
 *
 * @param[in] reg register
 *
 * @return offset on success. -1 on failure.
 */
static inline off_t __get_usr_reg_offset(usr_reg_name_t reg)
{
	switch ((usr_reg_name_t)reg) {
	case USRCC:
		return offsetof(core_user_reg_t, USRCC);
	case PSW:
		return offsetof(core_user_reg_t, PSW);
	case EXS:
		return offsetof(core_user_reg_t, EXS);
	case IC:
		return offsetof(core_user_reg_t, IC);
	case ICE:
		return offsetof(core_user_reg_t, ICE);
	case VIXR:
		return offsetof(core_user_reg_t, VIXR);
	case VL:
		return offsetof(core_user_reg_t, VL);
	case SAR:
		return offsetof(core_user_reg_t, SAR);
	case PMMR:
		return offsetof(core_user_reg_t, PMMR);
	default:
		if ((reg >= PMCR00) && (reg <= PMCR04))
			return offsetof(core_user_reg_t, PMCR[reg - PMCR00]);
		if ((reg >= PMC00) && (reg <= PMC15))
			return offsetof(core_user_reg_t, PMC[reg - PMC00]);
		if ((reg >= SR00) && (reg <= SR63))
			return offsetof(core_user_reg_t, SR[reg - SR00]);
	}
	return -1;
}

/**
 * @brief return offset of core_system_reg_t (internal function)
 *
 * @param[in] reg register
 *
 * @return offset on success. -1 on failure.
 */
static inline off_t __get_sys_reg_offset(sys_reg_name_t reg)
{
	switch (reg) {
	case EXSRAR:
		return offsetof(core_system_reg_t, EXSRAR);
	case HCR:
		return offsetof(core_system_reg_t, HCR);
	case JIDR:
		return offsetof(core_system_reg_t, JIDR);
	case DIDR:
		return offsetof(core_system_reg_t, DIDR);
	default:
		if ((reg >= CRD00) && (reg <= CRD03))
			return offsetof(core_system_reg_t, CRD[reg - CRD00]);
	}
	return -1;
}

/**
 * @brief get offset of specific register
 *
 * @param[in] type register type
 * @param[in] reg specific register
 *
 * @return offset on success. negative on failure.
 */
static inline off_t _get_reg_offset(reg_type_t type, int reg)
{
	off_t offset = -1;

	switch (type) {
	case USR_REG:
		offset = __get_usr_reg_offset((usr_reg_name_t)reg);
		break;
	case SYS_REG:
		offset = __get_sys_reg_offset((sys_reg_name_t)reg);
		break;
	}

	return offset;
}

#endif				/* if !defined(__LIBVED_INTERNAL_H) */
