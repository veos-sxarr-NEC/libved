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
#ifndef __VE_HW_H
#error "Never use <ve_hw_ve3.h> directly; include <ve_hw.h> instead."
#endif

#ifndef __VE_HW_VE3_H
#define __VE_HW_VE3_H

typedef union ve3_pciatb_entry {
	ve_reg_t data;
	struct {
		ve_reg_t rfu1:16, page_base_address:27, rfu2:13, sync:4 ,
			rfu3:2, cache_bypass:1, rfu4:1;
	} bf;
} ve3_pciatb_entry_t;

typedef union ve3_atb_entry {
	ve_reg_t data;
	struct {
		ve_reg_t rfu1:16, page_base_address:27, rfu2:9, type:3, rfu3:6, winh:1,
			cache_bypass:1, ua:1;
	} bf;
} ve3_atb_entry_t;

typedef union ve3_dmaatb_entry {
	ve_reg_t data;
	struct {
		ve_reg_t rfu1:12, page_base_address:40, type:3, rfu2:1, pci_attr:3,
			rfu3:2, winh:1, cache_bypass:1, ua:1;
	} bf;
} ve3_dmaatb_entry_t;

typedef union ve3_atb_dir {
	ve_reg_t data;
	struct {
		ve_reg_t rfu1:16, partial_space_address:19, rfu2:26,
			page_size:2, valid:1;
	} bf;
} ve3_atb_dir_t;

typedef union ve3_dmaatb_dir {
	ve_reg_t data;
	struct {
		ve_reg_t rfu1:16, partial_space_address:28, jid:8, rfu2:9,
			page_size:2, valid:1;
	} bf;
} ve3_dmaatb_dir_t;

typedef union ve3_dma_desc {
	ve_reg_t data[4];
	struct {
		ve_reg_t exc:16, rfu0:44, status:4;
		ve_reg_t rfu1:22, source:3, dest:3, control:4, length:32;
		ve_reg_t rfu2:16, source_addr:48;
		ve_reg_t rfu3:16, dest_addr:48;
	} dmd;
} ve3_dma_desc_t;

typedef union ve3_dma_control {
	ve_reg_t data[2];
	struct {
		ve_reg_t rfu0:62, stat:2;
		ve_reg_t exc:1, rfu1:56, read_p:7;
	} dcr;
} ve3_dma_control_t;

struct ve3_dma_control_pack {
	union ve3_dma_control DMACTL[2];
	ve_reg_t DMAJIDR[2];
	uint8_t padding[0x1000 - 0x30];
};

#define VE3_ATB_ENTRY_MAX_SIZE 256
#define VE3_SR_NUM 64 /*!< Number of Scalar Registers */
#define VE3_VR_NUM 64 /*!< Number of Vector Registers */
#define VE3_AUR_MVL 256 /*!< Maximum Length of Vector */
#define VE3_ATB_DIR_NUM 32 /*!< Number of ATB Directories */
#define VE3_PCIATB_EXT_SIZE 8192	/*!<
					 * Number of PCIATB Entries
					 * (reserved entries included)
					 */
#define VE3_PCIATB_VLD_SIZE 4096 /*!< Number of PCIATB Entries implented */
#define VE3_DMA_DSCR_MAX_INDEX 64
#define VE3_DMA_DSCR_ENTRY_SIZE 128
#define VE3_DMAATB_DIR_NUM 256 /*!< Numbedr of DMAATB Directories */
#define VE3_DMAATB_ENTRY_MAX_SIZE 256

#define VE3_PAGE_4KB 0
#define VE3_PAGE_2MB 1
#define VE3_PAGE_64MB 2
#define VE3_PAGE_256MB 3

/* EXS register */
#define CORRECTABLE_ERROR	((EXS_MONC)|(EXS_ADRM)|(EXS_BRTR)|\
		(EXS_STEP)|(EXS_RDBG)|0xFFULL)
#define UNCORRECTABLE_ERROR     (~(CORRECTABLE_ERROR))
#define UNKNOWN_ERROR           0x00000000FFFFFFFCULL
#define EXS_NOML        0x0000000000000000ULL /*!< Normal */
#define EXS_ILSP        0x8000000000000000ULL /*!<
					       * 00 Memory protection exception
					       */
#define EXS_ILMP        0x4000000000000000ULL /*!<
					       * 01 Missing page exception
					       */
#define EXS_ILMS        0x2000000000000000ULL /*!<
					       * 02 Missing space exception
					       */
#define EXS_ILSA        0x1000000000000000ULL /*!<
					       * 03 Memory access exception
					       */
#define EXS_DIVE        0x0080000000000000ULL /*!<
					       * 08 Division exception
					       */
#define EXS_FOFE        0x0040000000000000ULL /*!<
					       * 09 Floating-point overflow
					       * exception
					       */
#define EXS_FUFE        0x0020000000000000ULL /*!<
					       * 10 Floating-point underflow
					       * exception
					       */
#define EXS_XOFE        0x0010000000000000ULL /*!<
					       * 11 Fixed-point overflow
					       * exception
					       */
#define EXS_INVE        0x0008000000000000ULL /*!<
					       * 12 Invalid operation
					       * exception
					       */
#define EXS_INEE        0x0004000000000000ULL /*!<
					       * 13 Inexact exception
					       */
#define EXS_ILOP        0x0000400000000000ULL /*!<
					       * 17 Illegal instruction
					       * format exception
					       */
#define EXS_ILDT        0x0000200000000000ULL /*!<
					       * 18 Illegal data format
					       * exception
					       */
#define EXS_MONC        0x0000100000000000ULL /*!<
					       * 19 Software interrupt (MONC)
					       */
#define EXS_ADRM        0x0000080000000000ULL /*!<
					       * 20 Address match interrupt
					       */
#define EXS_BRTR        0x0000040000000000ULL /*!<
					       * 21 Branch trap
					       */
#define EXS_STEP        0x0000020000000000ULL /*!<
					       * 22 One step interrupt
					       */
#define EXS_MONT        0x0000010000000000ULL /*!<
					       * 23 Software interrupt
					       * (MONC TRAP)
					       */
#define EXS_HMILSP      0x0000008000000000ULL /*!<
					       * 24 Host memory protection
					       * exception
					       */
#define EXS_HMILMP      0x0000004000000000ULL /*!<
					       * 25 Host missing page exception
					       */
#define EXS_HMILMS      0x0000002000000000ULL /*!<
					       * 26 Host missing space exception
					       */
#define EXS_HMILSA      0x0000001000000000ULL /*!<
					       * 27 Host memory access exception
					       */
#define EXS_IOAC        0x0000000800000000ULL /*!<
					       * 28 I/O access exception
					       */
#define EXS_RDBG        0x0000000000800000ULL /*!<
					       * 40 Fast synchronization debug
					       * interrupt
					       */
#define EXS_STOPPING	0x3ULL
#define EXS_RUN		0xEEULL
#define USR_REG_SIZE	0x60000
#define EXS_STOP	0x1ULL

/* for DMAATB */
#define DMAATB_PB_OFFSET 9
#define DMAATB_VE_MEM_SPACE 0x3
#define DMAATB_VE_REG_SPACE 0x2
#define DMAATB_VH_MEM_SPACE 0x1

/* Core User Registers in BAR2 (prefetchable 64bit) */
typedef struct ve3_core_user_reg_pre {
	/* Performance Counters */
	ve_reg_t USRCC;
	ve_reg_t PMC[24];
	uint8_t pad0[4096 - (8 * (1+24))];
	/* Control Registers */
	ve_reg_t PSW;
	ve_reg_t Padding_exs;/* padding; EXS on VE1 */
	ve_reg_t IC;
	ve_reg_t ICE;
	ve_reg_t VIXR;
	ve_reg_t VL;
	ve_reg_t SAR;
	ve_reg_t PMMR;
	ve_reg_t PMCR[4];
	ve_reg_t PVL32;
	ve_reg_t PVL16;

	ve_reg_t pad1_0;
	uint8_t pad1[(5120-4096) - (8 * 15)];

	/* Scalar Registers */
	ve_reg_t SR[VE3_SR_NUM];
	uint8_t pad2[512];

	/* Vector Mask Registers */
	ve_reg_t VM[16][4];
	uint8_t pad3[1024 + 512];
	uint8_t pad3_1[(256*1024) - (2*4096)] ;

	/* Vector Registers */
	ve_reg_t VR[VE3_VR_NUM][VE3_AUR_MVL];
	uint8_t pad4[0x80000 - 0x60000] ;
} ve3_core_user_reg_pre_t;

/* Core User Registers in BAR4 (non-prefetchable 32bit) */
typedef struct ve3_core_user_reg_nonpre {
	ve_reg_t EXS  ; 
	uint8_t padding[0x1000 - 8];
} ve3_core_user_reg_nonpre_t;/* 4KB */

/* System Protection Register in BAR4 (non-prefetchable 32bit) */
typedef struct ve3_core_system_reg_nonpre {
	ve_reg_t EXSRAR; /**< Execution Status Report Address Register */
	uint8_t padding0[(0x1100-0x1008)];
	ve_reg_t HCR; /**< Hardware Configuration Register */
	uint8_t padding[(0x4020-0x1108)];
	ve_reg_t JIDR; /**< Job ID Register */
	ve_reg_t DIDR; /**< Debug ID Register */
	uint8_t padding2[(0xF000 - 0x4030)];
	uint8_t padding_a[(0x1000)];
} ve3_core_system_reg_nonpre_t;

/* ATB registers */
typedef struct ve3_atb_reg {
	union ve3_atb_entry entry[VE3_ATB_DIR_NUM][VE3_ATB_ENTRY_MAX_SIZE];
	union ve3_atb_dir dir[VE3_ATB_DIR_NUM];
} ve3_atb_reg_t;

/* System Protection Register in BAR2 (prefetchable 64bit) */
typedef struct ve3_core_system_reg_pre {
	ve3_atb_reg_t atb;
	ve_reg_t CRD[4]; /**< Communication Register Directory */

	uint8_t padding[(0x80000 - 0x10120)];
} ve3_core_system_reg_pre_t;

typedef struct ve3_cr_cluster {
	ve_reg_t data[32];
	uint8_t padding[0x1000-0x100];
} ve3_cr_cluster_t; 

typedef struct ve3_cr_cluster_bar_cr {
	ve_reg_t data[32];
	uint8_t padding[0x2000-0x100];
} ve3_cr_cluster_bar_cr_t;

typedef struct ve3_dmaatb_reg {
	union ve3_dmaatb_entry entry[VE3_DMAATB_DIR_NUM][VE3_DMAATB_ENTRY_MAX_SIZE];
	union ve3_dmaatb_dir dir[VE3_DMAATB_DIR_NUM];
} ve3_dmaatb_reg_t;

/* System Common Register in BAR2 (prefetchable 64bit) */
typedef struct ve3_system_common_reg_pre {
	union ve3_pciatb_entry pciatb[VE3_PCIATB_VLD_SIZE];
	uint8_t padding_0[0x10000-0x8000];

	ve_reg_t PCIATBA;

	uint8_t padding_1[0x40000-0x10008];

	ve3_dmaatb_reg_t dmaatb;
	uint8_t padding_2[0x100000-0xc0800];
	struct ve3_cr_cluster CR[32];

	uint8_t padding_3[0x200000 - 0x120000];

} ve3_system_common_reg_pre_t;

/* System Common Register in BAR4 (non-prefetchable 32bit) */
typedef struct ve3_system_common_reg_nonpre {
	ve_reg_t STM ;
	ve_reg_t SYNC ;
	ve_reg_t TPHBP;
	uint8_t padding_0[0x80-0x18];
	ve_reg_t TPHT[16];
	uint8_t padding_1[0x400-0x100];
	ve_reg_t PCIRCVSYC;
	uint8_t padding_2[0x0800-0x0408];

	ve_reg_t CPMC[256];
	ve_reg_t CPMMR[32];

	uint8_t padding_before_INTVEC[0x2000-0x1100];

	ve_reg_t INTVEC[4];

	uint8_t padding_before_PCIEXC[0x2400-0x2020];

	ve_reg_t PCIEXC[2];

	uint8_t padding_before_DMADESP[0x8000 - 0x2410];
	union ve3_dma_desc DMADESP[VE3_DMA_DSCR_ENTRY_SIZE]; 

	uint8_t padding_RFU_dmadesp[0xa000-0x9000];
	union ve3_dma_control DMACTLP;

	uint8_t padding_RFU_dmactlp[0x10000 - 0xa010];

	union ve3_dma_desc DMADES[VE3_DMA_DSCR_MAX_INDEX][VE3_DMA_DSCR_ENTRY_SIZE];

	struct ve3_dma_control_pack DMACTL_PACK[VE3_DMA_DSCR_MAX_INDEX/2 ];

	ve_reg_t PCISYAR[4];
	uint8_t padding_RFU_PCISYMR[0x070080-0x070020];
	ve_reg_t PCISYMR[4];

	uint8_t padding_pciatb_2[0x80000 - 0x0700a0];

	struct ve3_cr_cluster_bar_cr CR[32];
	uint8_t padding_RFU_cr[0x100000-0xa0000];
} ve3_system_common_reg_nonpre_t;

/* area #0 */
#define LIBVED_VE3_AREA_BAR23 (0)
#define LIBVED_VE3_AREA_PREFETCHABLE LIBVED_VE3_AREA_BAR23
typedef ve3_core_user_reg_pre_t ve3_core_user_reg_0_t;
typedef ve3_core_system_reg_pre_t ve3_core_system_reg_0_t;
typedef ve3_system_common_reg_pre_t ve3_system_common_reg_0_t;
/* area #1 */
#define LIBVED_VE3_AREA_BAR4 (1)
#define LIBVED_VE3_AREA_NON_PREFETCHABLE LIBVED_VE3_AREA_BAR4
typedef ve3_core_user_reg_nonpre_t ve3_core_user_reg_1_t;
typedef ve3_core_system_reg_nonpre_t ve3_core_system_reg_1_t;
typedef ve3_system_common_reg_nonpre_t ve3_system_common_reg_1_t;

/* Bit Mask for Arithmetic mode */
#define PSW_AM_MASK	(0xffffffffffffcfff)	/* RZ mode  */
#define PSW_AM_RZ	(0x0000000000000000)	/* Round towards zero*/
#define PSW_AM_RP	(0x0000000000001000)	/* Round towards plus infinity*/
#define PSW_AM_RM	(0x0000000000002000)	/* Round towards minus infinity*/
#define PSW_AM_RN	(0x0000000000003000)	/* Round to nearest even*/

#endif
