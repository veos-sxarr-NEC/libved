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
#ifndef __VE_HW_H
#error "Never use <ve_hw_ve1.h> directly; include <ve_hw.h> instead."
#endif

#ifndef __VE_HW_VE1_H
#define __VE_HW_VE1_H

/* registers */
typedef union ve1_pciatb_entry {
	ve_reg_t data;
	struct {
		ve_reg_t rfu1:16, page_base_address:27, rfu2:14, num:2, sync:1,
		      rfu3:2, cache_bypass:1, rfu4:1;
	} bf;
} ve1_pciatb_entry_t;

typedef union ve1_atb_entry {
	ve_reg_t data;
	struct {
		ve_reg_t rfu1:16, page_base_address:36, type:3, rfu2:6, winh:1,
		      cache_bypass:1, ua:1;
	} bf;
} ve1_atb_entry_t;

typedef union ve1_atb_dir_t {
	ve_reg_t data;
	struct {
		ve_reg_t rfu1:16, partial_space_address:28, jid:8, rfu2:9,
		      page_size:2, valid:1;
	} bf;
} ve1_atb_dir_t;

typedef union ve1_dma_desc {
	ve_reg_t data[4];
	struct {
		ve_reg_t exc:16, rfu0:44, status:4;
		ve_reg_t rfu1:21, source:3, rfu2:1, dest:3,
			control:4, length:32;
		ve_reg_t rfu3:16, source_addr:48;
		ve_reg_t rfu4:16, dest_addr:48;
	} dmd;
} ve1_dma_desc_t;

typedef union ve1_dma_control {
	ve_reg_t data[2];
	struct {
		ve_reg_t rfu0:62, stat:2;
		ve_reg_t rfu1:57, read_p:7;
	} dcr;
} ve1_dma_control_t;

#define VE1_ATB_DIR_NUM 32	/*!< Number of ATB Directory */
#define VE1_DMAATB_DIR_NUM 64	/*!< Number of DMAARB Directory */
#define VE1_SR_NUM 64		/*!< Number of Scalar Registers */
#define VE1_VR_NUM 64		/*!< Number of Vector Registers */
#define VE1_AUR_MVL 256		/*!< MAX Length of Vector */
#define VE1_PCIATB_VLD_SIZE 2048	/*!< Number of PCIATB Entries (valid) */
#define VE1_PCIATB_EXT_SIZE 8192	/*!<
					 * Number of PCIATB Entries (Extended)
					 */
#define VE1_ATB_ENTRY_MAX_SIZE 256	/*!< Number of ATB Entries */
#define VE1_DMAATB_ENTRY_MAX_SIZE 256	/*!< Number of DMAATB Entries */
#define VE1_DMA_DSCR_ENTRY_SIZE 128	/*!<
					 * Number of DMAATB Descriptor Entries
					 */

#define	VE1_PAGE_4K 0
#define	VE1_PAGE_2MB 1
#define	VE1_PAGE_64MB 2

/* EXS register */
#define CORRECTABLE_ERROR	((EXS_MONC)|(EXS_ADRM)|(EXS_BRTR)|\
		(EXS_STEP)|(EXS_RDBG)|0xFFULL)
#define UNCORRECTABLE_ERROR     (~(CORRECTABLE_ERROR))
#define UNKNOWN_ERROR		0x00000000FFFFFFFCULL
#define EXS_NOML	0x0000000000000000ULL /*!< Normal */
#define EXS_ILSP	0x8000000000000000ULL /*!<
					       * 00 Memory protection exception
					       */
#define EXS_ILMP	0x4000000000000000ULL /*!<
					       * 01 Missing page exception
					       */
#define EXS_ILMS	0x2000000000000000ULL /*!<
					       * 02 Missing space exception
					       */
#define EXS_ILSA	0x1000000000000000ULL /*!<
					       * 03 Memory access exception
					       */
#define EXS_DIVE	0x0080000000000000ULL /*!<
					       * 08 Division exception
					       */
#define EXS_FOFE	0x0040000000000000ULL /*!<
					       * 09 Floating-point overflow
					       * exception
					       */
#define EXS_FUFE	0x0020000000000000ULL /*!<
					       * 10 Floating-point underflow
					       * exception
					       */
#define EXS_XOFE	0x0010000000000000ULL /*!<
					       * 11 Fixed-point overflow
					       * exception
					       */
#define EXS_INVE	0x0008000000000000ULL /*!<
					       * 12 Invalid operation
					       * exception
					       */
#define EXS_INEE	0x0004000000000000ULL /*!<
					       * 13 Inexact exception
					       */
#define EXS_ILOP	0x0000400000000000ULL /*!<
					       * 17 Illegal instruction
					       * format exception
					       */
#define EXS_ILDT	0x0000200000000000ULL /*!<
					       * 18 Illegal data format
					       * exception
					       */
#define EXS_MONC	0x0000100000000000ULL /*!<
					       * 19 Software interrupt (MONC)
					       */
#define EXS_ADRM	0x0000080000000000ULL /*!<
					       * 20 Address match interrupt
					       */
#define EXS_BRTR	0x0000040000000000ULL /*!<
					       * 21 Branch trap
					       */
#define EXS_STEP	0x0000020000000000ULL /*!<
					       * 22 One step interrupt
					       */
#define EXS_MONT	0x0000010000000000ULL /*!<
					       * 23 Software interrupt
					       * (MONC TRAP)
					       */
#define EXS_HMILSP	0x0000008000000000ULL /*!<
					       * 24 Host memory protection
					       * exception
					       */
#define EXS_HMILMP	0x0000004000000000ULL /*!<
					       * 25 Host missing page exception
					       */
#define EXS_HMILMS	0x0000002000000000ULL /*!<
					       * 26 Host missing space exception
					       */
#define EXS_HMILSA	0x0000001000000000ULL /*!<
					       * 27 Host memory access exception
					       */
#define EXS_IOAC	0x0000000800000000ULL /*!<
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

/*
 * Register Map (Aurora ISA Draft2.4 + fix)
 */

/* User Registers (placed 512KB aligned address) */
typedef struct ve1_core_user_reg {
	/* Performance Counters */
	ve_reg_t USRCC;				/*     0x0 -     0x7 */
	ve_reg_t PMC[16];			/*     0x8 -    0x87 */
	uint8_t pad0[0x1000 - 0x88];		/*    0x88 -   0xFFF */
	/* Control Registers */
	ve_reg_t PSW;				/*  0x1000 -  0x1007 */
	ve_reg_t EXS;				/*  0x1008 -  0x100F */
	ve_reg_t IC;				/*  0x1010 -  0x1017 */
	ve_reg_t ICE;				/*  0x1018 -  0x101F */
	ve_reg_t VIXR;				/*  0x1020 -  0x1027 */
	ve_reg_t VL;				/*  0x1028 -  0x102F */
	ve_reg_t SAR;				/*  0x1030 -  0x1047 */
	ve_reg_t PMMR;				/*  0x1038 -  0x103F */
	ve_reg_t PMCR[4];			/*  0x1040 -  0x105F */
	uint8_t pad1[0x1400 - 0x1060];		/*  0x1060 -  0x13FF */
	/* Scalar Registers */
	ve_reg_t SR[VE1_SR_NUM];		/*  0x1400 -  0x15FF */
	uint8_t pad2[0x1800 - 0x1600];		/*  0x1600 -  0x17FF */
	/* Vector Mask Registers */
	ve_reg_t VMR[16][4];			/*  0x1800 -  0x19FF */
	uint8_t pad3[0x40000 - 0x1A00];		/*  0x1A00 - 0x3FFFF */
	/* Vector Registers */
	ve_reg_t VR[VE1_VR_NUM][VE1_AUR_MVL];	/* 0x40000 - 0x5FFFF */
	uint8_t pad4[0x80000 - 0x60000];	/* 0x60000 - 0x7FFFF */
} ve1_core_user_reg_t;

/* ATB register */
typedef struct ve1_atb_reg {
	ve1_atb_entry_t entry[VE1_ATB_DIR_NUM][VE1_ATB_ENTRY_MAX_SIZE];
						/*     0x0 - 0x0FFFF */
	ve1_atb_dir_t dir[VE1_ATB_DIR_NUM];	/* 0x10000 - 0x100FF */
} ve1_atb_reg_t;

/* System Registers (placed 512KB aligned address) */
typedef struct ve1_core_system_reg {
	/* ATB and Communication Register Directory */
	ve1_atb_reg_t atb;			/*     0x0 - 0x100FF */
	ve_reg_t CRD[4];			/* 0x10100 - 0x1011F */
	uint8_t pad0[0x10900 - 0x10120];	/* 0x10120 - 0x108FF */
	ve_reg_t EXSRAR;			/* 0x10900 - 0x10907 */
	uint8_t pad01[0x10A00 - 0x10908];	/* 0x10908 - 0x109FF */
	ve_reg_t HCR;				/* 0x10A00 - 0x10A07 */
	uint8_t pad02[0x20000 - 0x10A08];	/* 0x10A08 - 0x20000 */
	/* User DMA Descriptor */
	ve1_dma_desc_t dmades[2][VE1_DMA_DSCR_ENTRY_SIZE];
						/* 0x20000 - 0x21FFF */
	ve1_dma_control_t dmactl[2];		/* 0x22000 - 0x2201F */
	ve_reg_t JIDR;				/* 0x22020 - 0x22027 */
	ve_reg_t DIDR;				/* 0x22028 - 0x2202F */
	uint8_t pad1[0x80000 - 0x22030];	/* 0x22030 - 0x7FFFF */
} ve1_core_system_reg_t;

/* Communication Register (placed in system Common Registers) */
typedef struct ve1_cr_cluster {
	ve_reg_t data[32];		/*   0x0 -   0xFF */
	uint8_t pad[0x1000 - 0x100];	/* 0x100 -  0xFFF */
} ve1_cr_cluster_t;

/* DMAATB register */
typedef struct ve1_dmaatb_reg {
	ve1_atb_entry_t entry[VE1_DMAATB_DIR_NUM][VE1_DMAATB_ENTRY_MAX_SIZE];
						/*  0x80000 -  0x9FFFF */
	uint8_t pad0[0xC0000 - 0xA0000];	/*  0xA0000 -  0xBFFFF */
	ve1_atb_dir_t dir[VE1_DMAATB_DIR_NUM];	/*  0xC0000 -  0xC01FF */
	uint8_t pad1[0xC0400 - 0xC0200];	/*  0xC0200 -  0xC03FF */
} ve1_dmaatb_reg_t;

/* System Common Registers (placed 512KB aligned address) */
typedef struct ve1_system_common_reg {
	/* Common Registers */
	ve_reg_t STM;				/*      0x0 -      0x7 */
	ve_reg_t SYNC;				/*      0x8 -      0xF */
	ve_reg_t TPH_B;				/*     0x10 -     0x17 */
	uint8_t pad0[0x80 - 0x18];		/*     0x18 -     0x7F */
	ve_reg_t TPH_T[16];			/*     0x80 -     0xFF */
	uint8_t pad1[0x400 - 0x100];		/*    0x100 -    0x3FF */
	ve_reg_t PCIRCVSYC;			/*    0x400 -    0x407 */
	uint8_t pad2[0x800 - 0x408];		/*    0x408 -    0x7FF */
	ve_reg_t CPMC[256];			/*    0x800 -    0xFFF */
	ve_reg_t CPMMR[32];			/*   0x1000 -   0x10FF */
	uint8_t pad21[0x2000 - 0x1100];		/*   0x1100 -   0x1FFF */
	/* Interrupt Vector */
	ve_reg_t INTVEC;			/*   0x2000 -   0x2007 */
	uint8_t pad22[0x2400 - 0x2008];		/*   0x2008 -   0x23FF */
	ve_reg_t PCIEXC;				/*   0x2400 -   0x2407 */
	uint8_t pad3[0x40000 - 0x2408];		/*   0x2408 -  0x3FFFF */
	/* Privilege DMA Descriptor */
	ve1_dma_desc_t dmadesc[VE1_DMA_DSCR_ENTRY_SIZE];
						/*  0x40000 -  0x40FFF */
	uint8_t pad4[0x42000 - 0x41000];	/*  0x41000 -  0x41FFF */
	ve1_dma_control_t dmactl;		/*  0x42000 -  0x4200F */
	uint8_t pad5[0x60000 - 0x42010];	/*  0x42010 -  0x5FFFF */
	/* PCIATB */
	ve1_pciatb_entry_t pciatb[VE1_PCIATB_EXT_SIZE];
						/*  0x60000 -  0x6FFFF */
	ve_reg_t PCIATBA;			/*  0x70000 -  0x70007 */
	uint8_t pad51[0x70100 - 0x70008];	/*  0x70008 -  0x700FF */
	ve_reg_t pciatb_sync_addr[4];		/*  0x70100 -  0x7011F */
	uint8_t pad52[0x70180 - 0x70120];	/*  0x70120 -  0x7017F */
	ve_reg_t pciatb_sync_addr_mask[4];	/*  0x70180 -  0x7019F */
	uint8_t pad6[0x80000 - 0x701A0];        /*  0x701A0 -  0x7FFFF */
	/* DMAATB */
	ve1_dmaatb_reg_t dmaatb;		/*  0x80000 -  0xC03FF */
	uint8_t pad7[0x100000 - 0xC0400];	/*  0xC0400 -  0xFFFFF */
	/* Communication Registers */
	ve1_cr_cluster_t CR[32];		/* 0x100000 - 0x11FFFF */
	uint8_t pad8[0x200000 - 0x120000];	/* 0x120000 - 0x1FFFFF */
} ve1_system_common_reg_t;

/* Bit Mask for Arithmetic mode */
#define PSW_AM_MASK    (0xffffffffffffcfff)    /* RZ mode  */
#define PSW_AM_RZ      (0x0000000000000000)    /* Round towards zero*/
#define PSW_AM_RP      (0x0000000000001000)    /* Round towards plus infinity*/
#define PSW_AM_RM      (0x0000000000002000)    /* Round towards minus infinity*/
#define PSW_AM_RN      (0x0000000000003000)    /* Round to nearest even*/

#endif /* __VE_HW_H */
