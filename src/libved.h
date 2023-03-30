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
 * @file libved.h
 * @brief VE driver library header
 */

#if !defined(__LIBVED_H)
#define __LIBVED_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <stddef.h>
#include <sys/types.h>
#include <stdint.h>
#include <time.h>

/**
 * @brief handler structure for useing library functions.
 */
typedef uint64_t ve_reg_t;
struct vedl_handle_struct;
typedef struct vedl_handle_struct vedl_handle;

/* If you call vedl__wait_for_interrupt(), include <ve_drv.h>. */
struct ve_wait_irq;

struct ve_mem_info {
	uint64_t ve_mem_addr;
	uint64_t ve_mem_size;
};

typedef enum ve_usr_reg {
	VE_USR_USRCC,
	VE_USR_PMC00, VE_USR_PMC01, VE_USR_PMC02, VE_USR_PMC03,
	VE_USR_PMC04, VE_USR_PMC05, VE_USR_PMC06, VE_USR_PMC07,
	VE_USR_PMC08, VE_USR_PMC09, VE_USR_PMC10, VE_USR_PMC11,
	VE_USR_PMC12, VE_USR_PMC13, VE_USR_PMC14, VE_USR_PMC15,
	VE_USR_PSW,
	VE_USR_EXS,
	VE_USR_IC,
	VE_USR_ICE,
	VE_USR_VIXR,
	VE_USR_VL,
	VE_USR_SAR,
	VE_USR_PMMR,
	VE_USR_PMCR00, VE_USR_PMCR01, VE_USR_PMCR02, VE_USR_PMCR03,
	VE_USR__pad_0,/* for compatibilities with old libved */
	VE_USR_SR00, VE_USR_SR01, VE_USR_SR02, VE_USR_SR03, VE_USR_SR04,
	VE_USR_SR05, VE_USR_SR06, VE_USR_SR07, VE_USR_SR08, VE_USR_SR09,
	VE_USR_SR10, VE_USR_SR11, VE_USR_SR12, VE_USR_SR13, VE_USR_SR14,
	VE_USR_SR15, VE_USR_SR16, VE_USR_SR17, VE_USR_SR18, VE_USR_SR19,
	VE_USR_SR20, VE_USR_SR21, VE_USR_SR22, VE_USR_SR23, VE_USR_SR24,
	VE_USR_SR25, VE_USR_SR26, VE_USR_SR27, VE_USR_SR28, VE_USR_SR29,
	VE_USR_SR30, VE_USR_SR31, VE_USR_SR32, VE_USR_SR33, VE_USR_SR34,
	VE_USR_SR35, VE_USR_SR36, VE_USR_SR37, VE_USR_SR38, VE_USR_SR39,
	VE_USR_SR40, VE_USR_SR41, VE_USR_SR42, VE_USR_SR43, VE_USR_SR44,
	VE_USR_SR45, VE_USR_SR46, VE_USR_SR47, VE_USR_SR48, VE_USR_SR49,
	VE_USR_SR50, VE_USR_SR51, VE_USR_SR52, VE_USR_SR53, VE_USR_SR54,
	VE_USR_SR55, VE_USR_SR56, VE_USR_SR57, VE_USR_SR58, VE_USR_SR59,
	VE_USR_SR60, VE_USR_SR61, VE_USR_SR62, VE_USR_SR63,
	VE_USR_PVL32, VE_USR_PVL16,
	VE_USR_PMC16, VE_USR_PMC17, VE_USR_PMC18, VE_USR_PMC19, VE_USR_PMC20,
	/* following definitions reserve for future use */
	DMY_VE_USR_PMC21, DMY_VE_USR_PMC22, DMY_VE_USR_PMC23,
} ve_usr_reg_name_t;

typedef enum ve_sys_reg {
	VE_SYS_CRD00, VE_SYS_CRD01, VE_SYS_CRD02, VE_SYS_CRD03,
	VE_SYS_EXSRAR,
	VE_SYS_HCR,
	VE_SYS_JIDR,
	VE_SYS_DIDR,
} ve_sys_reg_name_t;

typedef enum vedl_interrupt_entry {
	VE_INTR_CORE00 = 0,
	VE_INTR_CORE01,
	VE_INTR_CORE02,
	VE_INTR_CORE03,
	VE_INTR_CORE04,
	VE_INTR_CORE05,
	VE_INTR_CORE06,
	VE_INTR_CORE07,
	VE_INTR_CORE08,
	VE_INTR_CORE09,
	VE_INTR_CORE10,
	VE_INTR_CORE11,
	VE_INTR_CORE12,
	VE_INTR_CORE13,
	VE_INTR_CORE14,
	VE_INTR_CORE15,
	VE_INTR_CORE16,
	VE_INTR_CORE17,
	VE_INTR_CORE18,
	VE_INTR_CORE19,
	VE_INTR_CORE20,
	VE_INTR_CORE21,
	VE_INTR_CORE22,
	VE_INTR_CORE23,
	VE_INTR_CORE24,
	VE_INTR_CORE25,
	VE_INTR_CORE26,
	VE_INTR_CORE27,
	VE_INTR_CORE28,
	VE_INTR_CORE29,
	VE_INTR_CORE30,
	VE_INTR_CORE31,

	VE_INTR_P_DMA,
	VE_INTR_PCI_ACCESS_EXC,

	VE_INTR_UDMA00,
	VE_INTR_CORE00_HDMA = VE_INTR_UDMA00,
	VE_INTR_UDMA01,
	VE_INTR_CORE00_EDMA = VE_INTR_UDMA01,
	VE_INTR_UDMA02,
	VE_INTR_CORE01_HDMA = VE_INTR_UDMA02,
	VE_INTR_UDMA03,
	VE_INTR_CORE01_EDMA = VE_INTR_UDMA03,
	VE_INTR_UDMA04,
	VE_INTR_CORE02_HDMA = VE_INTR_UDMA04,
	VE_INTR_UDMA05,
	VE_INTR_CORE02_EDMA = VE_INTR_UDMA05,
	VE_INTR_UDMA06,
	VE_INTR_CORE03_HDMA = VE_INTR_UDMA06,
	VE_INTR_UDMA07,
	VE_INTR_CORE03_EDMA = VE_INTR_UDMA07,
	VE_INTR_UDMA08,
	VE_INTR_CORE04_HDMA = VE_INTR_UDMA08,
	VE_INTR_UDMA09,
	VE_INTR_CORE04_EDMA = VE_INTR_UDMA09,
	VE_INTR_UDMA10,
	VE_INTR_CORE05_HDMA = VE_INTR_UDMA10,
	VE_INTR_UDMA11,
	VE_INTR_CORE05_EDMA = VE_INTR_UDMA11,
	VE_INTR_UDMA12,
	VE_INTR_CORE06_HDMA = VE_INTR_UDMA12,
	VE_INTR_UDMA13,
	VE_INTR_CORE06_EDMA = VE_INTR_UDMA13,
	VE_INTR_UDMA14,
	VE_INTR_CORE07_HDMA = VE_INTR_UDMA14,
	VE_INTR_UDMA15,
	VE_INTR_CORE07_EDMA = VE_INTR_UDMA15,
	VE_INTR_UDMA16,
	VE_INTR_CORE08_HDMA = VE_INTR_UDMA16,
	VE_INTR_UDMA17,
	VE_INTR_CORE08_EDMA = VE_INTR_UDMA17,
	VE_INTR_UDMA18,
	VE_INTR_CORE09_HDMA = VE_INTR_UDMA18,
	VE_INTR_UDMA19,
	VE_INTR_CORE09_EDMA = VE_INTR_UDMA19,
	VE_INTR_UDMA20,
	VE_INTR_CORE10_HDMA = VE_INTR_UDMA20,
	VE_INTR_UDMA21,
	VE_INTR_CORE10_EDMA = VE_INTR_UDMA21,
	VE_INTR_UDMA22,
	VE_INTR_CORE11_HDMA = VE_INTR_UDMA22,
	VE_INTR_UDMA23,
	VE_INTR_CORE11_EDMA = VE_INTR_UDMA23,
	VE_INTR_UDMA24,
	VE_INTR_CORE12_HDMA = VE_INTR_UDMA24,
	VE_INTR_UDMA25,
	VE_INTR_CORE12_EDMA = VE_INTR_UDMA25,
	VE_INTR_UDMA26,
	VE_INTR_CORE13_HDMA = VE_INTR_UDMA26,
	VE_INTR_UDMA27,
	VE_INTR_CORE13_EDMA = VE_INTR_UDMA27,
	VE_INTR_UDMA28,
	VE_INTR_CORE14_HDMA = VE_INTR_UDMA28,
	VE_INTR_UDMA29,
	VE_INTR_CORE14_EDMA = VE_INTR_UDMA29,
	VE_INTR_UDMA30,
	VE_INTR_CORE15_HDMA = VE_INTR_UDMA30,
	VE_INTR_UDMA31,
	VE_INTR_CORE15_EDMA = VE_INTR_UDMA31,
	VE_INTR_UDMA32,
	VE_INTR_UDMA33,
	VE_INTR_UDMA34,
	VE_INTR_UDMA35,
	VE_INTR_UDMA36,
	VE_INTR_UDMA37,
	VE_INTR_UDMA38,
	VE_INTR_UDMA39,
	VE_INTR_UDMA40,
	VE_INTR_UDMA41,
	VE_INTR_UDMA42,
	VE_INTR_UDMA43,
	VE_INTR_UDMA44,
	VE_INTR_UDMA45,
	VE_INTR_UDMA46,
	VE_INTR_UDMA47,
	VE_INTR_UDMA48,
	VE_INTR_UDMA49,
	VE_INTR_UDMA50,
	VE_INTR_UDMA51,
	VE_INTR_UDMA52,
	VE_INTR_UDMA53,
	VE_INTR_UDMA54,
	VE_INTR_UDMA55,
	VE_INTR_UDMA56,
	VE_INTR_UDMA57,
	VE_INTR_UDMA58,
	VE_INTR_UDMA59,
	VE_INTR_UDMA60,
	VE_INTR_UDMA61,
	VE_INTR_UDMA62,
	VE_INTR_UDMA63,

	VE_INTR_CORE00_FAULT,
	VE_INTR_CORE01_FAULT,
	VE_INTR_CORE02_FAULT,
	VE_INTR_CORE03_FAULT,
	VE_INTR_CORE04_FAULT,
	VE_INTR_CORE05_FAULT,
	VE_INTR_CORE06_FAULT,
	VE_INTR_CORE07_FAULT,
	VE_INTR_CORE08_FAULT,
	VE_INTR_CORE09_FAULT,
	VE_INTR_CORE10_FAULT,
	VE_INTR_CORE11_FAULT,
	VE_INTR_CORE12_FAULT,
	VE_INTR_CORE13_FAULT,
	VE_INTR_CORE14_FAULT,
	VE_INTR_CORE15_FAULT,
	VE_INTR_CORE16_FAULT,
	VE_INTR_CORE17_FAULT,
	VE_INTR_CORE18_FAULT,
	VE_INTR_CORE19_FAULT,
	VE_INTR_CORE20_FAULT,
	VE_INTR_CORE21_FAULT,
	VE_INTR_CORE22_FAULT,
	VE_INTR_CORE23_FAULT,
	VE_INTR_CORE24_FAULT,
	VE_INTR_CORE25_FAULT,
	VE_INTR_CORE26_FAULT,
	VE_INTR_CORE27_FAULT,
	VE_INTR_CORE28_FAULT,
	VE_INTR_CORE29_FAULT,
	VE_INTR_CORE30_FAULT,
	VE_INTR_CORE31_FAULT,

	VE_INTR_FAULT,
	VE_INTR_FAULT_DIAG,
	VE_INTR_FAULT_ENV,
	VE_INTR_HMC_ERR,
	VE_INTR_LLC_ERR,
	VE_INTR_MCU_ERR,
	VE_INTR_DP_ERR,
	VE_INTR_VR_ERR,
	VE_INTR_WARN_ENV,
	VE_INTR_DIAG_INFO,
	VE_INTR_MAX
} vedl_interrupt_entry_t;

struct vedl_user_reg_handle_struct;
typedef struct vedl_user_reg_handle_struct vedl_user_reg_handle;
struct vedl_sys_reg_handle_struct;
typedef struct vedl_sys_reg_handle_struct vedl_sys_reg_handle;
struct vedl_common_reg_handle_struct;
typedef struct vedl_common_reg_handle_struct vedl_common_reg_handle;

/**
 * @brief fence for mmapped register (before read)
 * @details
 * This function must be called before reading register
 * directly from the mmapped address.
 *
 * @param[in] handle VEDL handler
 */
static inline void vedl_barrier_regs_read(vedl_handle *handle)
{
	/* just prevent re-ordering */
	asm volatile ("":::"memory");
}

/**
 * @brief  fence for mmapped register (after write)
 * @details
 * This function must be called after writing register
 * directly to the mmapped address.
 *
 * @param[in] handle VEDL handle
 *
 * @return  0 on success. negative on failure.
 */
static inline void vedl_barrier_regs_write(vedl_handle *handle)
{
	/* just prevent re-ordering */
	asm volatile ("":::"memory");
}

void vedl_set_syscall_area_offset(vedl_handle *handle, off_t offset);
int vedl_get_syscall_area_offset(vedl_handle *handle);
void vedl_set_shm_lhm_addr(vedl_handle *handle, void *addr);
int vedl_get_syscall_num(vedl_handle *handle);

#define vedl_barrier_regs(handle) \
({ \
	int retval = 0; \
	vedl_barrier_regs_write(handle); \
	vedl_barrier_regs_read(handle); \
	retval; \
})

const char *vedl_get_sysfs_path(vedl_handle *);
uint64_t *vedl_get_shm_lhm_addr(const vedl_handle *);

const char *vedl_get_arch_class_name(const vedl_handle *);

size_t vedl_get_usr_reg_size(const vedl_user_reg_handle *reg, int area_id);
int vedl_get_usr_reg_words(const vedl_user_reg_handle *from,
		int area_id, off_t offset, void *to, size_t size);
int vedl_get_usr_reg(const vedl_user_reg_handle *from, ve_usr_reg_name_t reg,
		ve_reg_t *regdata);
int vedl_get_usr_reg_all(const vedl_user_reg_handle *from, int area_id,
				void *to, size_t size);
int vedl_set_usr_reg_words(vedl_user_reg_handle *to, int area_id, off_t offset,
		const void *from, size_t size);
int vedl_set_usr_reg(vedl_user_reg_handle *to, ve_usr_reg_name_t reg,
		ve_reg_t regdata);
int vedl_set_usr_reg_all(vedl_user_reg_handle *to, int area_id,
				const void *from, size_t size);
size_t vedl_get_sys_reg_size(const vedl_sys_reg_handle *reg, int area_id);
int vedl_get_sys_reg_words(const vedl_sys_reg_handle *from,
		int area_id, off_t offset, void *to, size_t size);
int vedl_get_sys_reg(const vedl_sys_reg_handle *from, ve_sys_reg_name_t reg,
		ve_reg_t *regdata);
int vedl_get_sys_reg_all(const vedl_sys_reg_handle *from, int area_id,
				void *to, size_t size);
int vedl_set_sys_reg_words(vedl_sys_reg_handle *to, int area_id, off_t offset,
		const void *from, size_t size);
int vedl_set_sys_reg(vedl_sys_reg_handle *to, ve_sys_reg_name_t reg,
		ve_reg_t regdata);
int vedl_set_sys_reg_all(vedl_sys_reg_handle *to, int area_id,
				const void *from, size_t size);

int vedl_get_cnt_reg_words(const vedl_common_reg_handle *from, int area_id,
		off_t offset, void *to, size_t size);
int vedl_get_cnt_reg_word(const vedl_common_reg_handle *from, int area_id,
		off_t offset, ve_reg_t *valp);
int vedl_set_cnt_reg_words(vedl_common_reg_handle *to, int area_id,
		off_t offset, const void *from, size_t size);
int vedl_set_cnt_reg_word(vedl_common_reg_handle *to, int area_id,
		off_t offset, ve_reg_t regdata);

vedl_handle *vedl_open_ve(const char *filename, int fd);
vedl_handle *vedl_request_new_handle(vedl_handle *p_handle, const char *fname);
int vedl_close_ve(vedl_handle *handle);
int vedl_get_syscall_args(vedl_handle *handle, uint64_t *args, int argnum);
int vedl_wait_exception(vedl_handle *handle, ve_reg_t *exs);
int vedl_create_ve_task(vedl_handle *handle, pid_t tid);
int vedl_delete_ve_task(vedl_handle *handle, pid_t tid);
int vedl_revive_ve_task(vedl_handle *handle, pid_t tid);
int vedl_delete_all_ve_task(vedl_handle *handle);
int vedl_assign_ve_task(vedl_handle *handle, int core_id, pid_t tid);
int vedl_unassign_ve_task(vedl_handle *handle, pid_t tid);
uint64_t vedl_get_dma_address(vedl_handle *handle, void *addr, pid_t pid,
		int pin_down, int write, int *pfnmap);
int vedl_get_addr_pin_blk(vedl_handle *handle, pid_t pid, uint64_t vaddr,
			  uint64_t *length, uint64_t *paddr, uint32_t maxpages,
			  uint32_t *npages, int *pgsz, int pin_down, int write);
uint64_t vedl_get_dma_address2(vedl_handle *handle, void *addr, pid_t pid,
		int pin_down, int write, int *pfnmap);
int vedl_reset_interrupt_count(vedl_handle *handle, uint64_t core_id);
int vedl_assign_cr(vedl_handle *handle, int cr_page, uid_t uid);
int vedl_unassign_cr(vedl_handle *handle, int cr_page, uid_t uid);
int vedl_assign_pci_vemem(vedl_handle *handle, int pciatb_entry, uid_t uid);
int vedl_unassign_pci_vemem(vedl_handle *handle, int pciatb_entry, uid_t uid);
int vedl_read_from_sysfs(vedl_handle *handle, const char *fname, char *str,
		int length);
int vedl_get_pci_bar_size(vedl_handle *handle, uint64_t *size, int bar);
int vedl_get_mem_info(vedl_handle *handle, struct ve_mem_info *mem_info);
long int vedl_get_num_of_core(vedl_handle *handle);
int vedl_get_pci_bar_address(vedl_handle *handle, uint64_t *addr, int bar);

int vedl_release_pindown_page(vedl_handle *handle, unsigned long addr);
int vedl_release_pindown_page2(vedl_handle *handle, unsigned long addr);
int vedl_release_pindown_page_all(vedl_handle *handle);
int vedl_release_pindown_page_all2(vedl_handle *handle);
int vedl_release_pindown_page_blk(vedl_handle *handle, uint64_t *addr,
				  int npages);
int vedl__wait_for_interrupt(vedl_handle *handle, struct ve_wait_irq *cond,
		struct timespec *timeout);
int vedl_wait_for_interrupt(vedl_handle *handle, vedl_interrupt_entry_t entry,
		struct timespec *timeout);
int vedl_wait_interrupt(vedl_handle *handle, int entry,
			 struct timespec *timeout);
vedl_user_reg_handle *vedl_mmap_usr_reg(vedl_handle *handle, int core_id);
vedl_sys_reg_handle *vedl_mmap_sys_reg(vedl_handle *handle, int core_id);
vedl_common_reg_handle *vedl_mmap_cnt_reg(vedl_handle *handle);
int vedl_munmap_usr_reg(vedl_user_reg_handle *user_reg_handle);
int vedl_munmap_sys_reg(vedl_sys_reg_handle *sys_reg_handle);
int vedl_munmap_common_reg(vedl_common_reg_handle *common_reg_handle);

int vedl_unmap_mmio(vedl_handle *handle, off_t offset, size_t size);

pid_t vedl_host_pid(vedl_handle *handle, pid_t host_pid, pid_t namespace_pid);
int vedl_get_fd(const vedl_handle *handle);
int vedl_reset_ve_chip(vedl_handle *handle, int sbr);
int vedl_reset_exsrar_mem(vedl_handle *handle, int core_id);

int vedl_update_firmware(vedl_handle *handle);

int vedl_notify_fault(vedl_handle *handle);
int vedl_request_ownership(vedl_handle *handle, int timeout);
int vedl_release_ownership(vedl_handle *handle);
int vedl_get_exs_register(vedl_handle *handle,ve_reg_t *exs);
int vedl_complete_memclear(vedl_handle *handle);
int vedl_handle_clock_gating(vedl_handle *handle, int flags);
int vedl_get_clock_gating_state(vedl_handle *handle, uint64_t *state);
#if defined(__cplusplus)
}
#endif
#endif				/* if !defined(__LIBVED_H) */


