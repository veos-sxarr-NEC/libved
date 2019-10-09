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
 * @file libved.h
 * @brief VE driver library header
 */

#if !defined(__LIBVED_H)
#define __LIBVED_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>
#include "ve_hw.h"

/**
 * @brief handler structure for useing library functions.
 */
struct vedl_handle_struct {
	int vefd;		/*!<
				 * vefd is device file descriptor.
				 * Device file descriptor have to be different
				 * among threads.
				 */
	uint64_t *lshm_addr;	/*!< LHM/SHM area */
	off_t offset;		/*!<
				 * syscall area offset from top of
				 * LHM/SHM area.
				 * This value must be updated at the time of
				 * thread/process creation.
				 */
	struct udev_device *udev;	/*!< sysfs udev device */
};

typedef struct vedl_handle_struct vedl_handle;

struct ve_mem_info {
	uint64_t ve_mem_addr;
	uint64_t ve_mem_size;
};

typedef enum usr_reg {
	USRCC,
	PMC00, PMC01, PMC02, PMC03, PMC04, PMC05, PMC06, PMC07, PMC08, PMC09,
	PMC10, PMC11, PMC12, PMC13, PMC14, PMC15,
	PSW,
	EXS,
	IC,
	ICE,
	VIXR,
	VL,
	SAR,
	PMMR,
	PMCR00, PMCR01, PMCR02, PMCR03, PMCR04,
	SR00, SR01, SR02, SR03, SR04, SR05, SR06, SR07, SR08, SR09, SR10, SR11,
	SR12, SR13, SR14, SR15, SR16, SR17, SR18, SR19, SR20, SR21, SR22, SR23,
	SR24, SR25, SR26, SR27, SR28, SR29, SR30, SR31, SR32, SR33, SR34, SR35,
	SR36, SR37, SR38, SR39, SR40, SR41, SR42, SR43, SR44, SR45, SR46, SR47,
	SR48, SR49, SR50, SR51, SR52, SR53, SR54, SR55, SR56, SR57, SR58, SR59,
	SR60, SR61, SR62, SR63,
} usr_reg_name_t;

typedef enum sys_reg {
	CRD00,
	CRD01,
	CRD02,
	CRD03,
	EXSRAR,
	HCR,
	JIDR,
	DIDR,
} sys_reg_name_t;

/**
 * @brief fence for mmapped register (before read)
 * @details
 * This function must be called before reading register
 * directly from the mmapped address.
 *
 * @param[in] handle VEDL handler
 * @param[in] addr core register mapped address
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

/**
 * @brief Set syscall area offset.
 *
 * @param handle VEDL handle
 * @param offset offset from top of LHM/SHM area.
 */
static inline void
vedl_set_syscall_area_offset(vedl_handle *handle, off_t offset)
{
	handle->offset = offset;
}

/**
 * @brief Get syscall area offset.
 *
 * @param handle VEDL handle
 */
static inline int vedl_get_syscall_area_offset(vedl_handle *handle)
{
	return handle->offset;
}

/**
 * @brief return SHM/LHM address
 *
 * @param[in] handle VEDL handler
 *
 * @return SHM/LHM address
 */
static inline void vedl_set_shm_lhm_addr(vedl_handle *handle, void *addr)
{
	handle->lshm_addr = (uint64_t *)addr;
}

/**
 * @brief return systemcall number
 *
 * @param[in] handle VEDL handler
 *
 * @return systemcall number on success. -1 on failure.
 */
static inline int vedl_get_syscall_num(vedl_handle *handle)
{
	return (*(uint64_t *)((char *)(handle->lshm_addr) + handle->offset));
}

/**
 * @brief get fd of VEDL handle
 *
 * @param[in] handle VEDL handle
 *
 * @return FD on success.
 */
static inline int vedl_get_fd(vedl_handle *handle)
{
	return handle->vefd;
}

#define vedl_barrier_regs(handle) \
({ \
	int retval = 0; \
	vedl_barrier_regs_write(handle); \
	vedl_barrier_regs_read(handle); \
	retval; \
})

/**
 * @brief get sysfs path associated with the handle
 * @param[in] handle pointer of VE handler
 *
 * @return sysfs path string address on success.
 *         NULL on failure.
 */
#define vedl_get_sysfs_path(handle) \
	udev_device_get_syspath(handle->udev)


#define vedl_get_shm_lhm_addr(handle) (handle->lshm_addr)

int vedl_get_usr_reg_words(vedl_handle *handle, core_user_reg_t *from,
		off_t offset, void *to, size_t size);
int vedl_get_usr_reg(vedl_handle *handle, core_user_reg_t *addr,
		usr_reg_name_t reg, reg_t *regdata);
int vedl_get_usr_reg_all(vedl_handle *handle, core_user_reg_t *from,
		core_user_reg_t *to);
int vedl_set_usr_reg_words(vedl_handle *handle, core_user_reg_t *to,
		off_t offset, void *from, size_t size);
int vedl_set_usr_reg(vedl_handle *handle, core_user_reg_t *to,
		usr_reg_name_t reg, reg_t regdata);
int vedl_set_usr_reg_all(vedl_handle *handle, core_user_reg_t *to,
		core_user_reg_t *from);
int vedl_get_sys_reg_words(vedl_handle *handle, core_system_reg_t *from,
		off_t offset, void *to, size_t size);
int vedl_get_sys_reg(vedl_handle *handle, core_system_reg_t *addr,
		sys_reg_name_t reg, reg_t *regdata);
int vedl_get_sys_reg_all(vedl_handle *handle, core_system_reg_t *from,
			 core_system_reg_t *to);
int vedl_set_sys_reg_words(vedl_handle *handle, core_system_reg_t *to,
		off_t offset, void *from, size_t size);
int vedl_set_sys_reg(vedl_handle *handle, core_system_reg_t *addr,
		sys_reg_name_t reg, reg_t regdata);
int vedl_set_sys_reg_all(vedl_handle *handle, core_system_reg_t *to,
			 core_system_reg_t *from);
int vedl_update_atb_all(vedl_handle *handle, core_system_reg_t *addr,
		atb_reg_t *atb);
int vedl_update_atb_dir(vedl_handle *handle, core_system_reg_t *addr,
		atb_reg_t *atb, int dirnum);
int vedl_update_atb_entry(vedl_handle *handle, core_system_reg_t *addr,
		atb_entry_t *entry, int dirnum, int entnum);
int vedl_get_cnt_reg_words(vedl_handle *handle, system_common_reg_t *from,
		off_t offset, void *to, size_t size);
int vedl_get_cnt_reg_word(vedl_handle *handle, system_common_reg_t *from,
		off_t offset, reg_t *valp);
int vedl_set_cnt_reg_words(vedl_handle *handle, system_common_reg_t *to,
		off_t offset, void *from, size_t size);
int vedl_set_cnt_reg_word(vedl_handle *handle, system_common_reg_t *to,
		off_t offset, reg_t val);
int vedl_get_dmaatb_all(vedl_handle *handle, system_common_reg_t *addr,
		dmaatb_reg_t *dmaatb);
int vedl_update_dmaatb_all(vedl_handle *handle, system_common_reg_t *addr,
		dmaatb_reg_t *dmaatb);
int vedl_update_dmaatb_dir(vedl_handle *handle, system_common_reg_t *addr,
		dmaatb_reg_t *dmaatb, int dirnum);
int vedl_update_dmaatb_entry(vedl_handle *handle, system_common_reg_t *addr,
		atb_entry_t *entry, int dirnum, int entnum);
int vedl_get_pciatb_all(vedl_handle *handle, system_common_reg_t *addr,
		pciatb_entry_t *pciatb);
int vedl_update_pciatb_all(vedl_handle *handle, system_common_reg_t *addr,
			   pciatb_entry_t *pciatb);
int vedl_update_pciatb_entry(vedl_handle *handle, system_common_reg_t *addr,
			     pciatb_entry_t *entry, int entnum);

vedl_handle *vedl_open_ve(const char *filename, int fd);
vedl_handle *vedl_request_new_handle(vedl_handle *p_handle, const char *fname);
int vedl_close_ve(vedl_handle *handle);
int vedl_get_syscall_args(vedl_handle *handle, uint64_t *args, int argnum);
int vedl_wait_exception(vedl_handle *handle, reg_t *exs);
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
#define vedl_get_pci_bar0_size(handle, addr) \
	vedl_get_pci_bar_size(handle, addr, 0)
int vedl_get_mem_info(vedl_handle *handle, struct ve_mem_info *mem_info);
long int vedl_get_num_of_core(vedl_handle *handle);
int vedl_get_pci_bar_address(vedl_handle *handle, uint64_t *addr, int bar);
#define vedl_get_pci_bar0_address(handle, addr) \
	vedl_get_pci_bar_address(handle, addr, 0)
#define vedl_get_pci_bar3_address(handle, addr) \
	vedl_get_pci_bar_address(handle, addr, 3)
int vedl_release_pindown_page(vedl_handle *handle, unsigned long addr);
int vedl_release_pindown_page2(vedl_handle *handle, unsigned long addr);
int vedl_release_pindown_page_all(vedl_handle *handle);
int vedl_release_pindown_page_all2(vedl_handle *handle);
int vedl_release_pindown_page_blk(vedl_handle *handle, uint64_t *addr,
				  int npages);
int vedl_wait_interrupt(vedl_handle *handle, msix_entry_t entry,
		struct timespec *timeout);
core_user_reg_t *vedl_mmap_usr_reg(vedl_handle *handle, int core_id);
core_system_reg_t *vedl_mmap_sys_reg(vedl_handle *handle, int core_id);
system_common_reg_t *vedl_mmap_cnt_reg(vedl_handle *handle);
int vedl_unmap_mmio(vedl_handle *handle, off_t offset, size_t size);
int vedl_update_firmware(vedl_handle *handle);
int vedl_reset_ve_chip(vedl_handle *handle, int sbr);
pid_t vedl_host_pid(vedl_handle *handle, pid_t host_pid, pid_t namespace_pid);
#if defined(__cplusplus)
}
#endif
#endif				/* if !defined(__LIBVED_H) */

