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

#if !defined(__LIBVED_REGISTER_H)
#define __LIBVED_REGISTER_H
#include <string.h>
#include <cpuid.h>

static inline void *rep_movs(void *to, const void *from, size_t n)
{
	asm volatile("rep ; movsq"
			: "=&c" (n), "=&D" (to), "=&S" (from)
			: "0" (n / 8), "q" (n), "1" ((uint64_t)to), "2" ((uint64_t)from)
			: "memory");
	return to;
}

static inline int is_ssse3()
{
	unsigned int a,b,c,d;
	static int bit_ssse3_on=0;
	static int check_once=1;

	if ( check_once ){
		a=b=c=d=0;
		__get_cpuid( 1, &a, &b, &c, &d);
		bit_ssse3_on = (c & bit_SSSE3);
		check_once = 0;
	}
	return bit_ssse3_on;
}

#define	MEMCP_TH_SIZE	0x800

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
 * @brief Check size and offset of MMIO of 1 word (8 bytes)
 *
 * @param offset offset from top address
 * @param size size to transfer
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
static inline int _check_size_offset_word(off_t offset, size_t size)
{
	/* size must be 8byte */
	if (size != 8)
		return -1;
	/* offset must be 8byte align */
	if (offset % 8 || offset < 0)
		return -2;
	return 0;
}

/**
 * @brief read register
 *
 * @param[in] from_addr register mapped address
 * @param offset offset from from_addr
 * @param[out] to_addr buffer for read
 * @param size size to read
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
static inline int _read_reg(const void *from_addr, off_t offset,
				void *to_addr, size_t size)
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
	if(size <= MEMCP_TH_SIZE && is_ssse3() ){
		memcpy(to_addr, (void *)(from_addr + offset), size);
	} else {
		rep_movs(to_addr, (void *)(from_addr + offset), size);
	}
#endif

	return 0;
}

/**
 * @brief read register 1 word
 *
 * @param[in] from_addr register mapped address
 * @param offset offset from from_addr
 * @param[out] to_addr buffer for read
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
static inline int _read_reg_word(const void *from_addr, off_t offset,
					ve_reg_t *to_addr)
{
	int err = 0;
	uint64_t *from8 = (uint64_t *)(from_addr + offset);

	err = _check_size_offset_word(offset, sizeof(uint64_t));
	if (err)
		return err;
	*to_addr = *from8;
	return 0;
}

/**
 * @brief write register 1 word
 *
 * @param[in] to_addr register mapped address
 * @param offset offset from to_addr
 * @param[in] from_addr buffer for write
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
static inline int _write_reg_word(void *to_addr, off_t offset,
					const ve_reg_t *from_addr)
{
	int err = 0;
	uint64_t *to8 = (uint64_t *)(to_addr + offset);

	err = _check_size_offset_word(offset, sizeof(uint64_t));
	if (err)
		return err;
	*to8 = *from_addr;
	return 0;
}

/**
 * @brief write register
 *
 * @param[in] to_addr register mapped address
 * @param offset offset from to_addr
 * @param[in] from_addr buffer for write
 * @param size size to write
 *
 * @return 0 on success.
 *         -1 on invalid size
 *         -2 on invalid offset
 */
static inline int _write_reg(void *to_addr, off_t offset,
				const void *from_addr, size_t size)
{
	int err = 0;
#ifdef NO_MEMCPY
	size_t now;
	uint64_t *to8 = (uint64_t *)(to_addr + offset);
	uint64_t *from8 = (uint64_t *)from_addr;
#else
	if (size == sizeof(uint64_t))
		return _write_reg_word(to_addr, offset, from_addr);
#endif

	err = _check_size_offset(offset, size);
	if (err)
		return err;
	/* write reg */
#ifdef NO_MEMCPY
	for(now = 0; now < size; now +=8)
		*to8++ = *from8++;	
#else
	if(size <= MEMCP_TH_SIZE && is_ssse3() ){
		memcpy((void *)(to_addr + offset), from_addr, size);
	} else {
		rep_movs((void *)(to_addr + offset), from_addr, size);
	}
#endif

	return 0;
}
#endif				/* if !defined(__LIBVED_REGISTER_H) */
