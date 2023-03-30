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

#if !defined(__LIBVED_INTERNAL_H)
#define __LIBVED_INTERNAL_H

#include <ve_drv.h>


struct vedl_arch_class;
/**
 * @brief handle structure for library functions.
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
	const struct vedl_arch_class *arch_class;/*!< VE architecture class */
};


struct mapped_area {
	void *start;
	size_t size;
};

struct reg_area_set {
	size_t num_areas;
	struct mapped_area *areas;
};

struct vedl_user_reg_handle_struct {
	struct reg_area_set user_regs;
	int (*get_usr_offset)(ve_usr_reg_name_t, int *, off_t *);
};
struct vedl_sys_reg_handle_struct {
	struct reg_area_set sys_regs;
	int (*get_sys_offset)(ve_sys_reg_name_t, int *, off_t *);
};

struct vedl_common_reg_handle_struct {
	struct reg_area_set common_regs;
	const void *arch_class;
};

/**
 * where to map register area; passed to mmap.
 */
struct reg_area {
	off_t offset;
	size_t size;
};

struct vedl_arch_class {
	char name[VEDRV_ARCH_CLASS_NAME_MAX];
	unsigned int bars_map;
	size_t num_user_regs_area;	/*!< # of areas of user regs */
	int (*get_user_regs_area)(int, int, struct reg_area *);
	int (*get_usr_offset)(ve_usr_reg_name_t, int *, off_t *);
	size_t num_sys_regs_area;	/*!< # of areas of sys regs */
	int (*get_sys_regs_area)(int, int, struct reg_area *);
	int (*get_sys_offset)(ve_sys_reg_name_t, int *, off_t *);
	size_t num_common_regs_area;	/*!< # of areas of common regs */
	int (*get_common_regs_area)(int, struct reg_area *);
	int (*create_wait_irq)(vedl_interrupt_entry_t, struct ve_wait_irq **);
	int (*update_firmware)(vedl_handle *handle);
	int (*wait_interrupt)(vedl_handle *handle, int msix_entry,
			      struct timespec *timeout);
	int (*request_ownership)(vedl_handle *handle, int timeout);
	int (*release_ownership)(vedl_handle *handle);
	int (*notify_fault)(vedl_handle *handle);
	int (*get_exs_register)(vedl_handle *handle, ve_reg_t *exs);
	int (*complete_memclear)(vedl_handle *handle);
	int (*handle_clock_gating)(vedl_handle *handle, int flags);
	int (*get_clock_gating_state)(vedl_handle *handle, uint64_t *state);
};

#define VEDL_ARCH_CLASS(name_) static const struct vedl_arch_class \
	* _arch_classes_ ##  name_ ## _ptr \
	__attribute__((__section__("arch_classes"), __used__)) = &(name_)
extern const struct vedl_arch_class *__start_arch_classes;
extern const struct vedl_arch_class *__stop_arch_classes;

#if defined(_VE_ARCH_VE3_)
#include "internal_ve3.h"
#elif defined(_VE_ARCH_VE1_)
#include "internal_ve1.h"

#endif
#endif				/* if !defined(__LIBVED_INTERNAL_H) */
