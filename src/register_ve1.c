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
 * @file  register_ve1.c
 * @brief VE1 specific register access functions
 */

#define _GNU_SOURCE
#include <errno.h>
#define _VE_ARCH_VE1_ (1)
#include "libved.h"
#include "ve_drv.h"
#include "ve_hw.h"
#include "internal.h"

int _vedl_ve1_get_user_regs_area(int core_id, int area_id,
					struct reg_area *ra)
{
	if (area_id != 0)
		return EINVAL;
	ra->offset = VEDRV_VE1_MAP_BAR2_OFFSET +
			VEDRV_VE1_PCI_BAR2_CREG_SIZE * core_id +
			VEDRV_VE1_PCI_BAR2_UREG_OFFSET;
	ra->size = VEDRV_VE1_PCI_BAR2_UREG_SIZE;
	return 0;
}

int _vedl_ve1_get_sys_regs_area(int core_id, int area_id,
					struct reg_area *ra)
{
	if (area_id != 0)
		return EINVAL;
	ra->offset = VEDRV_VE1_MAP_BAR2_OFFSET +
			VEDRV_VE1_PCI_BAR2_CREG_SIZE * core_id +
			VEDRV_VE1_PCI_BAR2_SREG_OFFSET;
	ra->size = VEDRV_VE1_PCI_BAR2_SREG_SIZE;
	return 0;
}

int _vedl_ve1_get_common_regs_area(int area_id, struct reg_area *ra)
{
	if (area_id != 0)
		return EINVAL;
	ra->offset = VEDRV_VE1_MAP_BAR2_OFFSET + VEDRV_VE1_PCI_BAR2_SCR_OFFSET;
	ra->size = VEDRV_VE1_PCI_BAR2_SCR_SIZE;
	return 0;
}

int _vedl_ve1_get_usr_offset(ve_usr_reg_name_t reg, int *areap, off_t *offp)
{
	switch (reg) {
#define VE1_CASE_USR_REG_OFFSET(REG_) \
	case (VE_USR_ ## REG_): do { \
		*areap = 0; \
		*offp = offsetof(ve1_core_user_reg_t, REG_); \
		return 0; } while (0)
	VE1_CASE_USR_REG_OFFSET(USRCC);
	VE1_CASE_USR_REG_OFFSET(PSW);
	VE1_CASE_USR_REG_OFFSET(EXS);
	VE1_CASE_USR_REG_OFFSET(IC);
	VE1_CASE_USR_REG_OFFSET(ICE);
	VE1_CASE_USR_REG_OFFSET(VIXR);
	VE1_CASE_USR_REG_OFFSET(VL);
	VE1_CASE_USR_REG_OFFSET(SAR);
	VE1_CASE_USR_REG_OFFSET(PMMR);
	default:
		if ((reg >= VE_USR_PMCR00) && (reg <= VE_USR_PMCR03)) {
			*areap = 0;
			*offp = offsetof(ve1_core_user_reg_t,
					PMCR[reg - VE_USR_PMCR00]);
			return 0;
		}
		if ((reg >= VE_USR_PMC00) && (reg <= VE_USR_PMC15)) {
			*areap = 0;
			*offp = offsetof(ve1_core_user_reg_t,
					PMC[reg - VE_USR_PMC00]);
			return 0;
		}
		if ((reg >= VE_USR_SR00) && (reg <= VE_USR_SR63)) {
			*areap = 0;
			*offp = offsetof(ve1_core_user_reg_t,
					SR[reg - VE_USR_SR00]);
			return 0;
		}
	}
	return -1;
}

int _vedl_ve1_get_sys_offset(ve_sys_reg_name_t reg, int *areap, off_t *offp)
{
	switch (reg) {
#define VE1_CASE_SYS_REG_OFFSET(REG_) \
	case (VE_SYS_ ## REG_): do { \
		*areap = 0; \
		*offp = offsetof(ve1_core_system_reg_t, REG_); \
		return 0; } while (0)
	VE1_CASE_SYS_REG_OFFSET(EXSRAR);
	VE1_CASE_SYS_REG_OFFSET(HCR);
	VE1_CASE_SYS_REG_OFFSET(JIDR);
	VE1_CASE_SYS_REG_OFFSET(DIDR);
	default:
		if ((reg >= VE_SYS_CRD00) && (reg <= VE_SYS_CRD03)) {
			*areap = 0;
			*offp = offsetof(ve1_core_system_reg_t,
					CRD[reg - VE_SYS_CRD00]);
			return 0;
		}
	}
	return -1;
}

#define CHECK_VE1_SYS_REG(reg) do { \
	if (reg->get_sys_offset != _vedl_ve1_get_sys_offset) \
		return -ENOTSUP; \
	} while (0)

/**
 * @brief update whole ATB (entire space)
 *
 * @param[in] reg core system register handle
 * @param[in] atb ATB data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_ve1_update_atb_all(vedl_sys_reg_handle *reg, const ve1_atb_reg_t *atb)
{
	CHECK_VE1_SYS_REG(reg);
	return vedl_set_sys_reg_words(reg, 0,
			offsetof(ve1_core_system_reg_t, atb), atb,
			sizeof(*atb));
}

/**
 * @brief update 1 ATB Space
 *
 * @param[in] reg core system register handle
 * @param[in] atb ATB data to store
 * @param dirnum ATB directory number
 *
 * @return 0 on success. negative on failure.
 *         -3 when dirnum is invalid
 */
int vedl_ve1_update_atb_dir(vedl_sys_reg_handle *reg, const ve1_atb_reg_t *atb,
			int dirnum)
{
	int ret;
	off_t atb_offset;
	off_t atb_dir_offset;
	off_t atb_dir_entry_offset;

	CHECK_VE1_SYS_REG(reg);
	if (dirnum >= VE1_ATB_DIR_NUM)
		return -3;

	atb_offset = offsetof(ve1_core_system_reg_t, atb);
	atb_dir_offset = atb_offset + offsetof(ve1_atb_reg_t, dir[dirnum]);
	atb_dir_entry_offset = atb_offset +
			offsetof(ve1_atb_reg_t, entry[dirnum]);

	/* update atb_dir_t */
	ret = vedl_set_sys_reg_words(reg, 0, atb_dir_offset,
			&atb->dir[dirnum], sizeof(ve1_atb_dir_t));
	if (ret)
		return ret;

	/* update atb_entry_t */
	ret = vedl_set_sys_reg_words(reg, 0, atb_dir_entry_offset,
		   atb->entry[dirnum],
		   sizeof(ve1_atb_entry_t) * VE1_ATB_ENTRY_MAX_SIZE);

	return ret;
}

/**
 * @brief update 1 ATB Entry
 *
 * @param[in] reg core system register handle
 * @param[in] entry ATB entry data to store
 * @param dirnum ATB directory number
 * @param entnum ATB entry number
 *
 * @return 0 on success. negative on failure.
 *         -3 when dirnum is invalid
 *         -4 when entnum is invalid
 */
int vedl_ve1_update_atb_entry(vedl_sys_reg_handle *reg,
		  const ve1_atb_entry_t *entry, int dirnum, int entnum)
{
	off_t atb_offset;
	off_t atb_entry_offset;

	CHECK_VE1_SYS_REG(reg);
	if (dirnum >= VE1_ATB_DIR_NUM)
		return -3;
	if (entnum >= VE1_ATB_ENTRY_MAX_SIZE)
		return -4;

	atb_offset = offsetof(ve1_core_system_reg_t, atb);
	atb_entry_offset =
	    atb_offset + offsetof(ve1_atb_reg_t, entry[dirnum][entnum]);

	/* update 1 entry */
	return vedl_set_sys_reg_words(reg, 0, atb_entry_offset, entry,
		   sizeof(ve1_atb_entry_t));
}

#define CHECK_VE1_CNT_REG(reg) do { \
	if (reg->arch_class != &_vedl_ve1_arch_class) \
		return -ENOTSUP; \
	} while (0)

/**
 * @brief get whole DMAATB
 *
 * @param[in] reg system common register handle
 * @param[out] dmaatb buffer for storing DMAATB data
 *
 * @return 0 on success. negative on failure.
 */
int vedl_ve1_get_dmaatb_all(const vedl_common_reg_handle *reg,
			ve1_dmaatb_reg_t *dmaatb)
{
	CHECK_VE1_CNT_REG(reg);
	return vedl_get_cnt_reg_words(reg, 0,
			offsetof(ve1_system_common_reg_t, dmaatb),
			dmaatb, sizeof(ve1_dmaatb_reg_t));
}

/**
 * @brief update whole DMAATB
 *
 * @param[in] reg system common register handle
 * @param[in] dmaatb DMAATB data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_ve1_update_dmaatb_all(vedl_common_reg_handle *reg,
			   const ve1_dmaatb_reg_t *dmaatb)
{
	CHECK_VE1_CNT_REG(reg);
	return vedl_set_cnt_reg_words(reg, 0,
			offsetof(ve1_system_common_reg_t, dmaatb),
			dmaatb, sizeof(ve1_dmaatb_reg_t));
}

/**
 * @brief update 1 DMAATB Space
 *
 * @param[in] reg system common register handle
 * @param[in] dmaatb DMAATB data to store
 * @param dirnum DMAATB directory number
 *
 * @return 0 on success. negative on failure.
 *         -3 when dirnum is invalid
 */
int vedl_ve1_update_dmaatb_dir(vedl_common_reg_handle *reg,
			   const ve1_dmaatb_reg_t *dmaatb, int dirnum)
{
	int ret;
	off_t atb_offset;
	off_t atb_dir_offset;
	off_t atb_dir_entry_offset;

	CHECK_VE1_CNT_REG(reg);
	if (dirnum >= VE1_DMAATB_DIR_NUM)
		return -3;

	atb_offset = offsetof(ve1_system_common_reg_t, dmaatb);
	atb_dir_offset = atb_offset + offsetof(ve1_dmaatb_reg_t, dir[dirnum]);
	atb_dir_entry_offset = atb_offset + offsetof(ve1_dmaatb_reg_t,
						     entry[dirnum]);

	/* update atb_dir_t */
	ret = vedl_set_cnt_reg_words(reg, 0, atb_dir_offset,
			&dmaatb->dir[dirnum], sizeof(ve1_atb_dir_t));
	if (ret)
		return ret;

	/* update atb_entry_t */
	ret = vedl_set_cnt_reg_words(reg, 0, atb_dir_entry_offset,
		   dmaatb->entry[dirnum],
		   sizeof(ve1_atb_entry_t) * VE1_DMAATB_ENTRY_MAX_SIZE);

	return ret;
}

/**
 * @brief update 1 DMAATB Entry
 *
 * @param[in] reg system common register handle
 * @param[in] entry DMAATB entry data to store
 * @param dirnum DMAATB directory number
 * @param entnum DMAATB entry number
 *
 * @return 0 on success. negative on failure.
 *         -3 when dirnum is invalid
 *         -4 when entnum is invalid
 */
int vedl_ve1_update_dmaatb_entry(vedl_common_reg_handle *reg,
				const ve1_atb_entry_t *entry,
				int dirnum, int entnum)
{
	off_t atb_offset;
	off_t atb_entry_offset;

	CHECK_VE1_CNT_REG(reg);
	if (dirnum >= VE1_DMAATB_DIR_NUM)
		return -3;
	if (entnum >= VE1_DMAATB_ENTRY_MAX_SIZE)
		return -4;

	atb_offset = offsetof(ve1_system_common_reg_t, dmaatb);
	atb_entry_offset = atb_offset + offsetof(ve1_dmaatb_reg_t,
						 entry[dirnum][entnum]);

	/* update 1 entry */
	return vedl_set_cnt_reg_words(reg, 0, atb_entry_offset, entry,
		   sizeof(ve1_atb_entry_t));
}

/**
 * @brief get whole PCIATB
 *
 * @param[in] reg system common register handle
 * @param[in] pciatb buffer for storing PCIATB
 *
 * @return 0 on success. negative on failure.
 */
int vedl_ve1_get_pciatb_all(const vedl_common_reg_handle *reg,
				ve1_pciatb_entry_t *pciatb)
{
	CHECK_VE1_CNT_REG(reg);
	return vedl_get_cnt_reg_words(reg, 0,
			offsetof(ve1_system_common_reg_t, pciatb), pciatb,
			sizeof(ve1_pciatb_entry_t) * VE1_PCIATB_VLD_SIZE);
}

/**
 * @brief update whole PCIATB
 *
 * @param[in] reg system common register handle
 * @param[in] pciatb PCIATB data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_ve1_update_pciatb_all(vedl_common_reg_handle *reg,
				const ve1_pciatb_entry_t *pciatb)
{
	CHECK_VE1_CNT_REG(reg);
	return vedl_set_cnt_reg_words(reg, 0,
			offsetof(ve1_system_common_reg_t, pciatb), pciatb,
			sizeof(ve1_pciatb_entry_t) * VE1_PCIATB_VLD_SIZE);
}

/**
 * @brief update 1 PCIATB Entry
 *
 * @param[in] reg system common register handle
 * @param[in] entry PCIATB entry data to store
 * @param entnum PCIATB entry number
 *
 * @return 0 on success. negative on failure.
 *         -3 when entnum is invalid
 */
int vedl_ve1_update_pciatb_entry(vedl_common_reg_handle *reg,
				const ve1_pciatb_entry_t *entry, int entnum)
{
	off_t atb_entry_offset;

	CHECK_VE1_CNT_REG(reg);
	if (entnum >= VE1_PCIATB_EXT_SIZE)
		return -3;

	atb_entry_offset = offsetof(ve1_system_common_reg_t, pciatb[entnum]);

	/* update 1 entry */
	return vedl_set_cnt_reg_words(reg, 0, atb_entry_offset, entry,
		   sizeof(ve1_atb_entry_t));
}
