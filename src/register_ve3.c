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
 * @file  register_ve3.c
 * @brief VE3 specific register access functions
 */

#define _GNU_SOURCE
#include <errno.h>
#define _VE_ARCH_VE3_ (1)
#include "libved.h"
#include "ve_drv.h"
#include "ve_hw.h"
#include "internal.h"

int _vedl_ve3_get_user_regs_area(int core_id, int area_id,
					struct reg_area *ra)
{
	switch (area_id) {
	case 0: /* BAR23 */
		ra->offset = VEDRV_VE3_MAP_BAR2_OFFSET +
			VEDRV_VE3_PCI_BAR2_CREG_SIZE * core_id +
			VEDRV_VE3_PCI_BAR2_UREG_OFFSET;
		ra->size = VEDRV_VE3_PCI_BAR2_UREG_SIZE;
		break;
	case 1: /* BAR4 */
		ra->offset = VEDRV_VE3_MAP_BAR4_OFFSET +
			VEDRV_VE3_PCI_BAR4_CREG_SIZE * core_id +
			VEDRV_VE3_PCI_BAR4_UREG_OFFSET;
		ra->size = VEDRV_VE3_PCI_BAR4_UREG_SIZE;
		break;
	default:
		return EINVAL;
	}
	return 0;
}

int _vedl_ve3_get_sys_regs_area(int core_id, int area_id,
					struct reg_area *ra)
{
	switch (area_id) {
	case 0: /* BAR23 */
		ra->offset = VEDRV_VE3_MAP_BAR2_OFFSET +
			VEDRV_VE3_PCI_BAR2_CREG_SIZE * core_id +
			VEDRV_VE3_PCI_BAR2_SREG_OFFSET;
		ra->size = VEDRV_VE3_PCI_BAR2_SREG_SIZE;
		break;
	case 1: /* BAR4 */
		ra->offset = VEDRV_VE3_MAP_BAR4_OFFSET +
			VEDRV_VE3_PCI_BAR4_CREG_SIZE * core_id +
			VEDRV_VE3_PCI_BAR4_SREG_OFFSET;
		ra->size = VEDRV_VE3_PCI_BAR4_SREG_SIZE;
		break;
	default:
		return EINVAL;
	}
	return 0;
}

int _vedl_ve3_get_common_regs_area(int area_id, struct reg_area *ra)
{
	switch (area_id) {
	case 0: /* BAR23 */
		ra->offset = VEDRV_VE3_MAP_BAR2_OFFSET +
			VEDRV_VE3_PCI_BAR2_SCR_OFFSET;
		ra->size = VEDRV_VE3_PCI_BAR2_SCR_SIZE;
		break;
	case 1: /* BAR4 */
		ra->offset = VEDRV_VE3_MAP_BAR4_OFFSET +
			VEDRV_VE3_PCI_BAR4_SCR_OFFSET;
		ra->size = VEDRV_VE3_PCI_BAR4_SCR_SIZE;
		break;
	default:
		return EINVAL;
	}
	return 0;
}

int _vedl_ve3_get_usr_offset(ve_usr_reg_name_t reg, int *areap, off_t *offp)
{
	switch (reg) {
#define VE3_CASE_USR_REG_PRE_OFFSET(REG_) \
	case (VE_USR_ ## REG_): do { \
		*areap = 0; /* ve3_core_user_reg_pre_t in BAR23 */ \
		*offp = offsetof(ve3_core_user_reg_pre_t, REG_); \
		return 0; } while (0)
	VE3_CASE_USR_REG_PRE_OFFSET(USRCC);
	VE3_CASE_USR_REG_PRE_OFFSET(PSW);
	VE3_CASE_USR_REG_PRE_OFFSET(IC);
	VE3_CASE_USR_REG_PRE_OFFSET(ICE);
	VE3_CASE_USR_REG_PRE_OFFSET(VIXR);
	VE3_CASE_USR_REG_PRE_OFFSET(VL);
	VE3_CASE_USR_REG_PRE_OFFSET(SAR);
	VE3_CASE_USR_REG_PRE_OFFSET(PMMR);
	VE3_CASE_USR_REG_PRE_OFFSET(PVL32);
	VE3_CASE_USR_REG_PRE_OFFSET(PVL16);
	case VE_USR_EXS:
		*areap = 1; /* ve3_core_user_reg_nonpre_t in BAR4 */
		*offp = offsetof(ve3_core_user_reg_nonpre_t, EXS);
		return 0;
	default:
		if ((reg >= VE_USR_PMCR00) && ( reg <= VE_USR_PMCR03)) {
			*areap = 0;
			*offp = offsetof(ve3_core_user_reg_pre_t,
					PMCR[reg - VE_USR_PMCR00]);
			return 0;
		}
		if ((reg >= VE_USR_PMC00) && (reg <= VE_USR_PMC15)) {
			*areap = 0;
			*offp = offsetof(ve3_core_user_reg_pre_t,
					PMC[reg - VE_USR_PMC00]);
			return 0;
		}
		if ((reg >= VE_USR_PMC16) && (reg <= VE_USR_PMC20)) {
			*areap = 0;
			*offp = offsetof(ve3_core_user_reg_pre_t,
					PMC[reg - VE_USR_PMC16 + 16]);
			return 0;
		}
		if ((reg >= VE_USR_SR00) && ( reg <= VE_USR_SR63)) {
			*areap = 0;
			*offp = offsetof(ve3_core_user_reg_pre_t,
					SR[reg - VE_USR_SR00]);
			return 0;
		}
	}
	return -1;
}

int _vedl_ve3_get_sys_offset(ve_sys_reg_name_t reg, int *areap, off_t *offp)
{
	switch (reg) {
#define VE3_CASE_SYS_REG_NONPRE_OFFSET(REG_) \
	case (VE_SYS_ ## REG_): do { \
		*areap = 1; /* ve_core_system_reg_nonpre_t in BAR4 */ \
		*offp = offsetof(ve3_core_system_reg_nonpre_t, REG_); \
		return 0; } while (0)
	VE3_CASE_SYS_REG_NONPRE_OFFSET(EXSRAR);
	VE3_CASE_SYS_REG_NONPRE_OFFSET(HCR);
	VE3_CASE_SYS_REG_NONPRE_OFFSET(JIDR);
	VE3_CASE_SYS_REG_NONPRE_OFFSET(DIDR);
	default:
		if ((reg >= VE_SYS_CRD00) && (reg <= VE_SYS_CRD03)) {
			*areap = 0; /* ve_core_system_reg_pre_t in BAR23 */
			*offp = offsetof(ve3_core_system_reg_pre_t,
					CRD[reg - VE_SYS_CRD00]);
			return 0;
		}
	}
	return -1;
}

#define CHECK_VE3_SYS_REG(reg) do { \
	if (reg->get_sys_offset != _vedl_ve3_get_sys_offset) \
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
int vedl_ve3_update_atb_all(vedl_sys_reg_handle *reg, const ve3_atb_reg_t *atb)
{
	CHECK_VE3_SYS_REG(reg);
	return vedl_set_sys_reg_words(reg, 0,
			offsetof(ve3_core_system_reg_pre_t, atb), atb,
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
int vedl_ve3_update_atb_dir(vedl_sys_reg_handle *reg, const ve3_atb_reg_t *atb,
			int dirnum)
{
	int ret;
	off_t atb_offset;
	off_t atb_dir_offset;
	off_t atb_dir_entry_offset;

	CHECK_VE3_SYS_REG(reg);
	if (dirnum >= VE3_ATB_DIR_NUM)
		return -3;

	atb_offset = offsetof(ve3_core_system_reg_pre_t, atb);
	atb_dir_offset = atb_offset + offsetof(ve3_atb_reg_t, dir[dirnum]);
	atb_dir_entry_offset = atb_offset +
			offsetof(ve3_atb_reg_t, entry[dirnum]);

	/* update atb_dir_t */
	ret = vedl_set_sys_reg_words(reg, 0, atb_dir_offset,
			&atb->dir[dirnum], sizeof(ve3_atb_dir_t));
	if (ret)
		return ret;

	/* update atb_entry_t */
	ret = vedl_set_sys_reg_words(reg, 0, atb_dir_entry_offset,
		   atb->entry[dirnum],
		   sizeof(ve3_atb_entry_t) * VE3_ATB_ENTRY_MAX_SIZE);

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
int vedl_ve3_update_atb_entry(vedl_sys_reg_handle *reg,
		  const ve3_atb_entry_t *entry, int dirnum, int entnum)
{
	off_t atb_offset;
	off_t atb_entry_offset;

	CHECK_VE3_SYS_REG(reg);
	if (dirnum >= VE3_ATB_DIR_NUM)
		return -3;
	if (entnum >= VE3_ATB_ENTRY_MAX_SIZE)
		return -4;

	atb_offset = offsetof(ve3_core_system_reg_pre_t, atb);
	atb_entry_offset =
	    atb_offset + offsetof(ve3_atb_reg_t, entry[dirnum][entnum]);

	/* update 1 entry */
	return vedl_set_sys_reg_words(reg, 0, atb_entry_offset, entry,
		   sizeof(ve3_atb_entry_t));
}

#define CHECK_VE3_CNT_REG(reg) do { \
	if (reg->arch_class != &_vedl_ve3_arch_class) \
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
int vedl_ve3_get_dmaatb_all(const vedl_common_reg_handle *reg,
			ve3_dmaatb_reg_t *dmaatb)
{
	CHECK_VE3_CNT_REG(reg);
	return vedl_get_cnt_reg_words(reg, 0,
			offsetof(ve3_system_common_reg_pre_t, dmaatb),
			dmaatb, sizeof(ve3_dmaatb_reg_t));
}

/**
 * @brief update whole DMAATB
 *
 * @param[in] reg system common register handle
 * @param[in] dmaatb DMAATB data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_ve3_update_dmaatb_all(vedl_common_reg_handle *reg,
			   const ve3_dmaatb_reg_t *dmaatb)
{
	CHECK_VE3_CNT_REG(reg);
	return vedl_set_cnt_reg_words(reg, 0,
			offsetof(ve3_system_common_reg_pre_t, dmaatb),
			dmaatb, sizeof(ve3_dmaatb_reg_t));
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
int vedl_ve3_update_dmaatb_dir(vedl_common_reg_handle *reg,
			   const ve3_dmaatb_reg_t *dmaatb, int dirnum)
{
	int ret;
	off_t atb_offset;
	off_t atb_dir_offset;
	off_t atb_dir_entry_offset;

	CHECK_VE3_CNT_REG(reg);
	if (dirnum >= VE3_DMAATB_DIR_NUM)
		return -3;

	atb_offset = offsetof(ve3_system_common_reg_pre_t, dmaatb);
	atb_dir_offset = atb_offset + offsetof(ve3_dmaatb_reg_t, dir[dirnum]);
	atb_dir_entry_offset = atb_offset + offsetof(ve3_dmaatb_reg_t,
						     entry[dirnum]);

	/* update atb_dir_t */
	ret = vedl_set_cnt_reg_words(reg, 0, atb_dir_offset,
			&dmaatb->dir[dirnum], sizeof(ve3_atb_dir_t));
	if (ret)
		return ret;

	/* update atb_entry_t */
	ret = vedl_set_cnt_reg_words(reg, 0, atb_dir_entry_offset,
		   dmaatb->entry[dirnum],
		   sizeof(ve3_atb_entry_t) * VE3_DMAATB_ENTRY_MAX_SIZE);

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
int vedl_ve3_update_dmaatb_entry(vedl_common_reg_handle *reg,
				const ve3_dmaatb_entry_t *entry,
				int dirnum, int entnum)
{
	off_t atb_offset;
	off_t atb_entry_offset;

	CHECK_VE3_CNT_REG(reg);
	if (dirnum >= VE3_DMAATB_DIR_NUM)
		return -3;
	if (entnum >= VE3_DMAATB_ENTRY_MAX_SIZE)
		return -4;

	atb_offset = offsetof(ve3_system_common_reg_pre_t, dmaatb);
	atb_entry_offset = atb_offset + offsetof(ve3_dmaatb_reg_t,
						 entry[dirnum][entnum]);

	/* update 1 entry */
	return vedl_set_cnt_reg_words(reg, 0, atb_entry_offset, entry,
		   sizeof(ve3_atb_entry_t));
}

/**
 * @brief get whole PCIATB
 *
 * @param[in] reg system common register handle
 * @param[in] pciatb buffer for storing PCIATB
 *
 * @return 0 on success. negative on failure.
 */
int vedl_ve3_get_pciatb_all(const vedl_common_reg_handle *reg,
				ve3_pciatb_entry_t *pciatb)
{
	CHECK_VE3_CNT_REG(reg);
	return vedl_get_cnt_reg_words(reg, 0,
			offsetof(ve3_system_common_reg_pre_t, pciatb), pciatb,
			sizeof(ve3_pciatb_entry_t) * VE3_PCIATB_VLD_SIZE);
}

/**
 * @brief update whole PCIATB
 *
 * @param[in] reg system common register handle
 * @param[in] pciatb PCIATB data to store
 *
 * @return 0 on success. negative on failure.
 */
int vedl_ve3_update_pciatb_all(vedl_common_reg_handle *reg,
				const ve3_pciatb_entry_t *pciatb)
{
	CHECK_VE3_CNT_REG(reg);
	return vedl_set_cnt_reg_words(reg, 0,
			offsetof(ve3_system_common_reg_pre_t, pciatb), pciatb,
			sizeof(ve3_pciatb_entry_t) * VE3_PCIATB_VLD_SIZE);
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
int vedl_ve3_update_pciatb_entry(vedl_common_reg_handle *reg,
				const ve3_pciatb_entry_t *entry, int entnum)
{
	off_t atb_entry_offset;

	CHECK_VE3_CNT_REG(reg);
	if (entnum >= VE3_PCIATB_EXT_SIZE)
		return -3;

	atb_entry_offset = offsetof(ve3_system_common_reg_pre_t,
					pciatb[entnum]);

	/* update 1 entry */
	return vedl_set_cnt_reg_words(reg, 0, atb_entry_offset, entry,
		   sizeof(ve3_atb_entry_t));
}
