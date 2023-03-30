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
 * @file  libved.c
 * @brief VE driver library
 */

#define _GNU_SOURCE
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <libudev.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <limits.h>
#include "ve_drv.h"
#include "libved.h"
#include "internal.h"

/**
 * \addtogroup VEDL API
 *
 * VE Driver API functions.
 * To use VE Driver API functions, include "libved.h" header.
 */
//@{
/**
 * @brief Set syscall area offset.
 *
 * @param[in] handle VEDL handle
 * @param offset SHM area offset assigned to thread
 */
void vedl_set_syscall_area_offset(vedl_handle *handle, off_t offset)
{
	handle->offset = offset;
}

/**
 * @brief Get syscall area offset.
 *
 * @param[in] handle VEDL handle
 * @return offset SHM area offset assigned to thread
 */
int vedl_get_syscall_area_offset(vedl_handle *handle)
{
	return handle->offset;
}

/**
 * @brief Set SHM/LHM address
 *
 * @param[in] handle VEDL handler
 * @param addr SHM/LHM address
 *
 */
void vedl_set_shm_lhm_addr(vedl_handle *handle, void *addr)
{
	handle->lshm_addr = (uint64_t *)addr;
}

/**
 * @brief Get systemcall number
 *
 * @param[in] handle VEDL handler
 *
 * @return systemcall number on success. -1 on failure.
 */
int vedl_get_syscall_num(vedl_handle *handle)
{
	return (*(uint64_t *)((char *)(handle->lshm_addr) + handle->offset));
}

/**
 * @brief Get sysfs path associated with the handle
 * @param[in] handle pointer of VE handler
 *
 * @return sysfs path string address on success.
 *         NULL on failure.
 */
const char *vedl_get_sysfs_path(vedl_handle *handle)
{
	return udev_device_get_syspath(handle->udev);
}
/**
 * @brief Get SHM/LHM address
 *
 * @param[in] handle VEDL handler
 * @return SHM/LHM address
 *
 */
uint64_t *vedl_get_shm_lhm_addr(const vedl_handle *handle)
{
	return handle->lshm_addr;
}

const struct vedl_arch_class *find_arch_class(char *name)
{
	char *s;
	const struct vedl_arch_class **p;
	/* remove \n */
	s = strchr(name, '\n');
	if (s)
		*s = '\0';
	for (p = &__start_arch_classes; p < &__stop_arch_classes; ++p)
		if (strcmp((*p)->name, name) == 0)
			return *p;
	return NULL;
}

/**
 * @brief Aquire VE Driver Library handler
 *
 * @param[in] filename Device file of VE driver.
 * @param fd Negative to open the file newly.
 *           Otherwise fd is used for handle.
 *
 * @return handler structure pointer on success.
 *         NULL on failure. (errno will be saved)
 */
vedl_handle *vedl_open_ve(const char *filename, int fd)
{
	vedl_handle *handle;
	int errsv = 0;
	int err;
	struct stat sb;
	struct udev *udev = udev_new();
	char abi_buff[10];
	char arch_class_buff[VEDRV_ARCH_CLASS_NAME_MAX];
	long int abi_version;

	handle = (vedl_handle *)malloc(sizeof(vedl_handle));
	if (handle == NULL) {
		errsv = errno;
		fprintf(stderr, "%s vedl_handle\n", strerror(errno));
		goto malloc_err;
	}
	if (fd >= 0) {
		handle->vefd = fd;
	} else {
		handle->vefd = open(filename, O_RDWR);
		if (handle->vefd == -1) {
			errsv = errno;
			fprintf(stderr, "%s %s\n", strerror(errno), filename);
			goto open_err;
		}
	}

	err = fstat(handle->vefd, &sb);
	if (err == -1) {
		errsv = errno;
		fprintf(stderr, "%s %s\n", strerror(errno), filename);
		goto fstat_err;
	}
	handle->udev = udev_device_new_from_devnum(udev, 'c', sb.st_rdev);
	if (handle->udev == NULL) {
		errsv = errno;
		fprintf(stderr, "%s udev device doesn't exists.\n",
				strerror(errno));
		goto udev_err;
	}

	/* Check driver ABI version */
	err = vedl_read_from_sysfs(handle, "abi_version", abi_buff, 10);
	if (err) {
		errsv = errno;
		fprintf(stderr, "%s VE Driver ABI version check error.\n",
				strerror(errno));
		goto abi_err;
	}
	errno = 0;
	abi_version = strtol(abi_buff, NULL, 10);
	if ((errno == ERANGE
			&& (abi_version == LONG_MAX || abi_version == LONG_MIN))
			|| (errno != 0 && abi_version == 0)) {
		goto abi_err;
	}
	if (abi_version != VEDRV_ABI_VERSION) {
		errsv = errno;
		fprintf(stderr, "VE Driver and libved version mismatch.\n"
				"VE Driver ABI: %ld, libved ABI: %d\n",
				abi_version, VEDRV_ABI_VERSION);
		goto abi_err;
	}
	/* Find VE architecture class */
	memset(arch_class_buff, 0, sizeof(arch_class_buff));
	err = vedl_read_from_sysfs(handle, "ve_arch_class", arch_class_buff,
					sizeof(arch_class_buff) - 1);
	if (err) {
		errsv = errno;
		fprintf(stderr, "%s cannot find VE architecture class.\n",
			strerror(errno));
		goto find_arch_err;
	}
	handle->arch_class = find_arch_class(arch_class_buff);
	if (handle->arch_class == NULL) {
		fprintf(stderr, "libved does not support VE %s\n", filename);
		goto find_arch_err;
	}

	return handle;

 find_arch_err:
 abi_err:
	udev_device_unref(handle->udev);
 udev_err:
 fstat_err:
	close(handle->vefd);
 open_err:
	free(handle);
 malloc_err:
	errno = errsv;
	udev_unref(udev);
	return NULL;
}

/**
 * @brief request new handle for multi-threading/processing
 *
 * @param[in] p_handle parent's handle
 * @param[in] fname Device file of VE driver
 *                  NULL to inherit from parent's fd
 *
 * @return handler structure on success. NULL on failure.
 */
vedl_handle *vedl_request_new_handle(vedl_handle *p_handle, const char *fname)
{
	vedl_handle *new_handle;
	char proclnk[PATH_MAX];
	struct udev *udev;
	int ret;
	int errsv = 0;

	new_handle = (vedl_handle *)malloc(sizeof(vedl_handle));
	if (new_handle == NULL)
		return NULL;

	/* copy handle */
	memcpy(new_handle, p_handle, sizeof(vedl_handle));
	udev = udev_device_get_udev(new_handle->udev);
	if(udev == NULL)
		goto err_get_udev;
	udev_device_ref(new_handle->udev);
	udev_ref(udev);

	if (fname)
		ret = snprintf(proclnk, PATH_MAX, "%s", fname);
	else
		ret = snprintf(proclnk, PATH_MAX, "/proc/self/fd/%d",
				p_handle->vefd);
	if ((ret < 0) | (PATH_MAX < ret)) {
		errsv = errno;
		goto err_snprintf;
	}

	new_handle->vefd = open(proclnk, O_RDWR);
	if (new_handle->vefd == -1) {
		errsv = errno;
		fprintf(stderr, "%s %s\n", proclnk, strerror(errno));
		goto err_open;
	}

	return new_handle;

 err_open:
 err_snprintf:
	udev_unref(udev);
	udev_device_unref(new_handle->udev);
 err_get_udev:
	free(new_handle);
	errno = errsv;
	return NULL;
}

/**
 * @brief Close VEDL handle
 *
 * @param[in] handle VEDL handle
 *
 * @return 0 on success.
 *         -1 on failure.
 */
int vedl_close_ve(vedl_handle *handle)
{
	int ret;
	struct udev *udev; 
	
	ret = close(handle->vefd);
	if (ret)
		goto err;

	udev = udev_device_get_udev(handle->udev);
	udev_unref(udev);
	udev_device_unref(handle->udev);
	free(handle);
 err:
	return ret;
}

/**
 * @brief Get VE architecture class name
 *
 * @param[in] handle VEDL handle
 *
 * @return VE architecture class name
 */
const char *vedl_get_arch_class_name(const vedl_handle *handle)
{
	return handle->arch_class->name;
}

/**
 * @brief Get syscall arguments
 *
 * @param[in] handle VEDL handle
 * @param[out] args argument lists
 * @param argnum number of arguments to put to *args
 *
 * @return 0 on success.
 *         -2 on argnum is too big.
 */
int vedl_get_syscall_args(vedl_handle *handle, uint64_t *args, int argnum)
{
	if (argnum > 6)
		return -2;
	memcpy(args, (void *)handle->lshm_addr + handle->offset +
	       sizeof(uint64_t), sizeof(uint64_t) * argnum);

	return 0;
}

/**
 * @brief Wait for VE program exception
 * @details This function returns when VE core generate any exceptions.
 * System call is one of exceptions.
 *
 * @param[in] handle VEDL handle
 * @param[out] exs EXS register value
 *
 * @return 0 on success.
 *         negative on failure.
 *         errno:
 *           EFAULT  The VE task does not exist
 *           ESRCH   The VE task ID is different from caller thread ID
 *           EAGAIN  O_NONBLOCK is specified and the caller
 *                   is going to sleep
 *           EINTR   Interrupted by signal
 */
int vedl_wait_exception(vedl_handle *handle, ve_reg_t *exs)
{
	return ioctl(handle->vefd, VEDRV_CMD_WAIT_EXCEPTION, exs);
}

/**
 * @brief Create VE task structure on vedriver
 *
 * @param[in] handle VEDL handle
 * @param tid VE task id
 *
 * @return 0 on success.
 *         negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           ESRCH   The VE task does not exists
 */
int vedl_create_ve_task(vedl_handle *handle, pid_t tid)
{
	return ioctl(handle->vefd, VEDRV_CMD_CREATE_TASK, tid);
}

/**
 * @brief Delete VE task structure on VE driver
 *
 * @param[in] handle VEDL handle
 * @param tid VE process TID
 *
 * @return 0 on success.
 *         negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           EAGAIN  The VE task is currently assigned to core
 *           ESRCH   The VE task does not exist
 */
int vedl_delete_ve_task(vedl_handle *handle, pid_t tid)
{
	return ioctl(handle->vefd, VEDRV_CMD_DELETE_TASK, tid);
}

/**
 * @brief Revive VE task which is deleted on vedriver
 *
 * @param[in] handle VEDL handle
 * @param tid VE task id
 *
 * @return 0 on success.
 *         negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           ESRCH   The VE task does not exists
 *           EINVAL  The VE task is in non-revival state
 */
int vedl_revive_ve_task(vedl_handle *handle, pid_t tid)
{
	return ioctl(handle->vefd, VEDRV_CMD_REVIVE_TASK, tid);
}


/**
 * @brief Delete all VE task structure on VE driver
 *
 * @param[in] handle VEDL handle
 *
 * @return 0 on success.
 *         negative on failure.
 *         errno:
 *           EPERM  The caller is not permitted
 */
int vedl_delete_all_ve_task(vedl_handle *handle)
{
	return ioctl(handle->vefd, VEDRV_CMD_DELETE_ALL_TASK);
}

/**
 * @brief Assign VE task to VE core on VE driver
 *
 * @param[in] handle VEDL handle
 * @param core_id VE physical core ID
 * @param tid VE task TID which is assigned to the core
 *
 * @return 0 on success.
 *         negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           EINVAL  The core_id is invalid
 *           ESRCH   The VE task does not exist
 *           EBUSY   The VE task is already assigned to another core
 *           EAGAIN  The Exception of the task is not handled yet
 */
int vedl_assign_ve_task(vedl_handle *handle, int core_id, pid_t tid)
{
	int ret = -1;
	struct ve_tid_core arg;

	arg.tid = tid;
	arg.core_id = core_id;

	ret = ioctl(handle->vefd, VEDRV_CMD_ASSIGN_TASK, &arg);

	return ret;
}

/**
 * @brief Unassign VE task from VE core
 *
 * @param[in] handle VEDL handler
 * @param tid VE process TID which is unassigned from the core
 *
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           ESRCH   The tid is invalid
 *           EPERM   The caller is not permitted
 *           EBUSY   The core is busy
 *           EINVAL  The tid is not assigned
 *           EAGAIN  Exception interrupt is not arrived yet
 */
int vedl_unassign_ve_task(vedl_handle *handle, pid_t tid)
{
	return ioctl(handle->vefd, VEDRV_CMD_UNASSIGN_TASK, tid);
}

static uint64_t do_get_dma_address(vedl_handle *handle, enum pd_list list_type,
		void *addr, pid_t pid, int pin_down, int write, int *pfnmap)
{
	int retval;
	struct ve_vp ve_arg;

	ve_arg.type = list_type;
	ve_arg.vp_info.virt = (uint64_t)addr;
	ve_arg.vp_info.pid = pid;
	ve_arg.vp_info.write = write;
	if (pin_down)
		retval = ioctl(handle->vefd, VEDRV_CMD_VHVA_TO_VSAA_PIN_DOWN,
				&ve_arg);
	else
		retval = ioctl(handle->vefd, VEDRV_CMD_VHVA_TO_VSAA, &ve_arg);
	if (retval)
		return retval;

	if (pfnmap != NULL)
		*pfnmap = ve_arg.vp_info.pfnmap;

	return ve_arg.vp_info.phys;
}

/**
 * @brief return physical address of process virtual address
 *
 * @param[in] handle VEDL handle
 * @param[in] addr virtual address to translate
 * @param pid PID to translate
 * @param pin_down set 1 to pin down the page on the memory
 * @param write set 1 to check if the page is writable
 * @param[out] pfnmap function will set 1 if the address is IO mapped address
 *
 * @return physical address on success.
 *         (uint64_t)-1 on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           ESRCH   Invalid PID
 *           ENOMEM  No enough memory or failed to pin down page
 */
uint64_t vedl_get_dma_address(vedl_handle *handle, void *addr, pid_t pid,
		int pin_down, int write, int *pfnmap)
{
	return do_get_dma_address(handle, OS_LIST, addr, pid, pin_down,
			write, pfnmap);
}

/** 
 * @brief bulk translate virtual addresses to physical addresses
 *
 * @param[in] handle 	VEDL handle
 * @param[in] pid 	PID to translate
 * @param[in] vaddr 	virtual addresses to translate
 * @param[inout] length	pointer to length of pin downed memory
 * @param[out] paddr	pointer to the list of physical addresses
 * @param[in] maxpages	the upper limit of the number of pin downed pages
 * @param[out] npages	pointer to the number of pin downed pages
 * @param[out] pgsz	pointer to the memory page size  
 * @param[in] pin_down 	set 1 to pin down the page on the memory
 * @param[in] write 	set 1 to check if the page is writable
 *
 * @return zero on success
 *         negative value on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           ESRCH   Invalid PID
 *           ENOMEM  No enough memory or failed to pin down page
 */
int vedl_get_addr_pin_blk(vedl_handle *handle, pid_t pid, uint64_t vaddr,
			  uint64_t *length, uint64_t *paddr, uint32_t maxpages,
			  uint32_t *npages, int *pgsz, int pin_down, int write)
{
	int retval;
	struct ve_vp_blk ve_arg;

	ve_arg.type = OS_LIST;
	ve_arg.vp_info.pid = pid;
	ve_arg.vp_info.write = write;
	ve_arg.vp_info.vaddr = vaddr;
	ve_arg.vp_info.length = *length;
	ve_arg.vp_info.paddr = paddr;
	ve_arg.vp_info.maxpages = maxpages;
	if (pin_down)
		retval = ioctl(handle->vefd, VEDRV_CMD_VHVA_TO_VSAA_BLK_PIN_DOWN,
				&ve_arg);
	else
		retval = ioctl(handle->vefd, VEDRV_CMD_VHVA_TO_VSAA_BLK, &ve_arg);
	if (retval)
		return retval;
	*pgsz = ve_arg.vp_info.pgsz;
	*length = ve_arg.vp_info.length;
	*npages = ve_arg.vp_info.npages;
	return 0;
}

/**
 * @brief Get physical address of process virtual address for MMM, T&D
 *
 * @param[in] handle VEDL handle
 * @param[in] addr virtual address to translate
 * @param pid PID to translate
 * @param pin_down set 1 to pin down the page on the memory
 * @param write set 1 to check if the page is writable
 * @param[out] pfnmap function will set 1 if the address is IO mapped address
 *
 * @return physical address on success.
 *         (uint64_t)-1 on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           ESRCH   Invalid PID
 *           ENOMEM  No enough memory or failed to pin down page
 */
uint64_t vedl_get_dma_address2(vedl_handle *handle, void *addr, pid_t pid,
		int pin_down, int write, int *pfnmap)
{
	return do_get_dma_address(handle, TD_LIST, addr, pid, pin_down,
			write, pfnmap);
}

/**
 * @brief Reset interrupt count of core
 *
 * @param[in] handle VEDL handle
 * @param core_id Physical VE Core ID
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *	     EINVAL  Invalid core_id.
 */
int vedl_reset_interrupt_count(vedl_handle *handle, uint64_t core_id)
{
	return ioctl(handle->vefd, VEDRV_CMD_RST_INTR_COUNT, core_id);
}

/**
 * @brief Assign CR to UID
 *
 * @param[in] handle VEDL handle
 * @param cr_page CR page number
 * @param uid UID to assign
 *
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           EINVAL  Invalid cr_page, Invalid uid
 *           ENOMEM  No enough memory
 */
int vedl_assign_cr(vedl_handle *handle, int cr_page, uid_t uid)
{
	int retval = -1;
	struct ve_cr_assign arg;

	arg.cr_page_num = cr_page;
	arg.owner_uid = uid;

	retval = ioctl(handle->vefd, VEDRV_CMD_ASSIGN_CR, &arg);

	return retval;
}

/**
 * @brief Unassign CR
 *
 * @param[in] handle VEDL handle
 * @param cr_page CR page number
 * @param uid UID to unassign
 *
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           EINVAL  Invalid cr_page, Invalid uid
 */
int vedl_unassign_cr(vedl_handle *handle, int cr_page, uid_t uid)
{
	int retval = -1;
	struct ve_cr_assign arg;

	arg.cr_page_num = cr_page;
	arg.owner_uid = uid;

	retval = ioctl(handle->vefd, VEDRV_CMD_UNASSIGN_CR, &arg);

	return retval;
}

/**
 * @brief Assign PCI VE memory to UID
 *
 * @param[in] handle VEDL handle
 * @param pciatb_entry PCI ATB entry number
 * @param uid UID to assign
 *
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           EINVAL  Invalid pciatb_entry, Invalid uid
 *           ENOMEM  No enought memory
 */
int vedl_assign_pci_vemem(vedl_handle *handle, int pciatb_entry, uid_t uid)
{
	int retval = -1;
	struct ve_pcimem_assign arg;

	arg.pciatb_entry = pciatb_entry;
	arg.owner_uid = uid;

	retval = ioctl(handle->vefd, VEDRV_CMD_ASSIGN_VEMEM, &arg);

	return retval;
}

/**
 * @brief Unassign PCI VE memory from UID
 *
 * @param[in] handle VEDL handle
 * @param pciatb_entry PCI ATB entry number
 * @param uid UID to unassign
 *
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *           EINVAL  Invalid pciatb_entry, Invalid uid
 */
int vedl_unassign_pci_vemem(vedl_handle *handle, int pciatb_entry, uid_t uid)
{
	int retval = -1;
	struct ve_pcimem_assign arg;

	arg.pciatb_entry = pciatb_entry;
	arg.owner_uid = uid;

	retval = ioctl(handle->vefd, VEDRV_CMD_UNASSIGN_VEMEM, &arg);

	return retval;
}

/**
 * @brief Read string from sysfs file
 *
 * @param[in] handle VEDL handle
 * @param[in] fname sysfs file name
 * @param[out] str String read from sysfs
 * @param length size of "str"
 *
 * @return 0 on success. Negative on failure.
 *         errno will be saved.
 */
int vedl_read_from_sysfs(vedl_handle *handle, const char *fname, char *str,
		int length)
{
	int fd;
	int errsv;
	int retval, readlen;
	char string[PATH_MAX];

	retval = snprintf(string, PATH_MAX, "%s/%s",
			vedl_get_sysfs_path(handle), fname);
	if (retval < 0)
		return retval;

	fd = open(string, O_RDONLY);
	if (fd == -1)
		return retval;

 again:
	readlen = read(fd, str, length - 1);
	if (readlen < 0) {
		if (errno == EINTR)
			goto again;
		else {
			errsv = errno;
			close(fd);
			errno = errsv;
			return readlen;
		}
	}
	str[readlen] = '\0';
	retval = close(fd);

	return retval;
}

static int bar_valid(const vedl_handle *handle, int bar)
{
	if (bar < 0 || 6 <= bar)
		return 0;
	return (handle->arch_class->bars_map & (1 << bar)) != 0;
}


/**
 * @brief Get size of PCI BAR
 *
 * @param[in] handle VEDL handle
 * @param[out] size size of BAR will be written here
 * @param bar BAR number (currently BAR0 only)
 *
 * @return 0 on success.
 *         -EINVAL on invalid BAR number.
 *         -EIO on failure.
 *         -ERANGE on out of range.
 */
int vedl_get_pci_bar_size(vedl_handle *handle, uint64_t *size, int bar)
{
	char string[PATH_MAX];
	char filename[sizeof("bar0_size")];
	int retval;

	if (!bar_valid(handle, bar))
		return -EINVAL;

	snprintf(filename, sizeof(filename), "bar%d_size", bar);

	retval = vedl_read_from_sysfs(handle, filename, string, PATH_MAX);
	if (retval)
		return -EIO;

	errno = 0;
	*size = strtoull(string, NULL, 0);
	if (*size == ULLONG_MAX && errno == ERANGE)
		return -ERANGE;

	return 0;
}

/**
 * @brief Get VE memory information
 *
 * @param[in] handle VEDL handle
 * @param[out] mem_info memory information structure
 *
 * @return 0 on success.
 *         -EIO on failure.
 *         -ERANGE on out of range
 */
int vedl_get_mem_info(vedl_handle *handle, struct ve_mem_info *mem_info)
{
	char string[PATH_MAX];
	int retval;

	retval = vedl_read_from_sysfs(handle, "memory_size", string, PATH_MAX);
	if (retval)
		return -EIO;

	errno = 0;
	retval = strtoul(string, NULL, 0);
	if (retval == ULONG_MAX && errno == ERANGE)
		return -ERANGE;
	mem_info->ve_mem_size = (uint64_t)retval * 1024 * 1024 * 1024;
	mem_info->ve_mem_addr = 0;

	return 0;
}

/**
 * @brief Get number of core
 *
 * @param[in] handle VEDL handle
 *
 * @return number of core on success.
 *         -EIO on failure.
 *         -ERANGE on out of range
 */
long int vedl_get_num_of_core(vedl_handle *handle)
{
	char string[PATH_MAX];
	long int retval;

	retval = vedl_read_from_sysfs(handle, "num_of_core", string, PATH_MAX);
	if (retval)
		return -EIO;

	errno = 0;
	retval = strtol(string, NULL, 0);
	if ((errno == ERANGE && (retval == LONG_MAX || retval == LONG_MIN))
			|| (errno != 0 && retval == 0))
		return -ERANGE;

	return retval;
}

/**
 * @brief Get address of PCI BAR
 *
 * @param[in] handle VEDL handle
 * @param[out] addr address of BAR will be written here
 * @param bar BAR number
 *
 * @return 0 on success.
 *         -EINVAL on invalid bar number.
 *         -EIO on failure.
 *         -ERANGE on out of range
 */
int vedl_get_pci_bar_address(vedl_handle *handle, uint64_t *addr, int bar)
{
	char string[PATH_MAX];
	char filename[sizeof("bar0_addr")];
	int retval;

	if (!bar_valid(handle, bar))
		return -EINVAL;

	snprintf(filename, sizeof(filename), "bar%d_addr", bar);
	retval = vedl_read_from_sysfs(handle, filename, string, PATH_MAX);
	if (retval)
		return -EIO;

	errno = 0;
	*addr = strtoull(string, NULL, 0);
	if (*addr == ULLONG_MAX && errno == ERANGE)
		return -ERANGE;

	return 0;
}

static int do_release_pindown_page(vedl_handle *handle, enum pd_list list_type,
		unsigned long addr)
{
	struct ve_vp_release arg;

	arg.type = list_type;
	arg.addr = addr;

	return ioctl(handle->vefd, VEDRV_CMD_RELEASE_PD_PAGE, &arg);
}


/**
 * @brief Release (count down) pinned down page
 *
 * @param[in] handle VEDL handler
 * @param addr physical address of the page
 *
 * @return 0 on success. negative on failure.
 *         errno:
 *           EINVAL Invalid address.
 *           ESRCH  The page is not pinned
 */
int vedl_release_pindown_page(vedl_handle *handle, unsigned long addr)
{
	return do_release_pindown_page(handle, OS_LIST, addr);
}

/**
 * @brief Release (count down) pinned down page for MMM, T&D
 *
 * @param[in] handle VEDL handler
 * @param addr physical address of the page
 *
 * @return 0 on success. negative on failure.
 *         errno:
 *           EINVAL Invalid address.
 *           ESRCH  The page is not pinned
 */
int vedl_release_pindown_page2(vedl_handle *handle, unsigned long addr)
{
	return do_release_pindown_page(handle, TD_LIST, addr);
}

static int do_release_pindown_page_all(vedl_handle *handle,
		enum pd_list list_type)
{
	struct ve_vp_release arg;

	arg.type = list_type;
	arg.addr = -1;

	return ioctl(handle->vefd, VEDRV_CMD_RELEASE_PD_PAGE_ALL, &arg);
}

/**
 * @brief Release (count down) all pinned down page
 *
 * @param[in] handle VEDL handler
 *
 * @return 0 on success. negative on failure.
 */
int vedl_release_pindown_page_all(vedl_handle *handle)
{
	return do_release_pindown_page_all(handle, OS_LIST);
}

/**
 * @brief Release (count down) all pinned down page for MMM, T&D
 *
 * @param[in] handle VEDL handler
 *
 * @return 0 on success. negative on failure.
 */
int vedl_release_pindown_page_all2(vedl_handle *handle)
{
	return do_release_pindown_page_all(handle, TD_LIST);
}

/**
 * @brief Bulk release (count down) pinned down pages
 *
 * @param[in] handle VEDL handler
 * @param addr pointer to list of physical (page) addresses
 * @param npages number of pages in list
 *
 * @return 0 on success. negative on failure.
 *         errno:
 *           EINVAL Invalid address.
 *           ESRCH  The page is not pinned
 */
int vedl_release_pindown_page_blk(vedl_handle *handle, uint64_t *addr,
				  int npages)
{
	struct ve_vp_blk_release arg;

	arg.type = OS_LIST;
	arg.addr = addr;
	arg.npages = npages;

	return ioctl(handle->vefd, VEDRV_CMD_RELEASE_PD_PAGE_BLK, &arg);
}

/**
 * @brief wait for interrupt
 *
 * @details
 * This function block until specified interrupt.
 *
 * @param[in] handle VEDL handle
 * @param[in] cond low-level structure to specify an interrupt.
 * @param[in] timeout timeout count
 *
 * @return 0 on success; netative on failure with errno set to:
 *           ETIMEDOUT time expired;
 *           EFAULT    a pointer in arguments is bad;
 *           EINTR     interrupted by a signal.
 */
int vedl__wait_for_interrupt(vedl_handle *handle, struct ve_wait_irq *cond,
			struct timespec *timeout)
{
	int retval = 0;
	struct ve_wait_irq_arg arg;
	arg.bits = cond;
	arg.timeout = timeout;

	retval = ioctl(handle->vefd, VEDRV_CMD_WAIT_INTR, &arg);
	if (retval > 0)
		return 0;
	return retval;
}

/**
 * @brief wait for an interupt
 *
 * @details
 * This function blocks until a specified entry interrupt occur.
 *
 * @param[in] handle VEDL handler
 * @param entry interrupt entry number to wait
 * @param[in] timeout timeout count
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           ETIMEDOUT time expired.
 *	     EFAULT    invalid argument.
 *	     EINTR     receiving signal.
 */
int vedl_wait_for_interrupt(vedl_handle *handle, vedl_interrupt_entry_t entry,
			struct timespec *timeout)
{
	int retval = 0;
	struct ve_wait_irq *cond;

	retval = handle->arch_class->create_wait_irq(entry, &cond);
	if (retval != 0) {
		errno = retval;
		return -1;
	}
	retval = vedl__wait_for_interrupt(handle, cond, timeout);
	free(cond);
	return retval;
}

static int init_reg_area_set(struct reg_area_set *rs, int n)
{
	rs->num_areas = n;
	rs->areas = calloc(n, sizeof(*rs->areas));
	if (rs->areas == 0)
		return ENOMEM;
	return 0;
}

static int map_core_reg_area_set(int fd, int core_id, struct reg_area_set *rs,
			int (*get_area)(int, int, struct reg_area *))
{
	int i;
	int rv = 0;
	for (i = 0; i < rs->num_areas; ++i) {
		struct reg_area ra;
		rs->areas[i].start = MAP_FAILED;
		(void)get_area(core_id, i, &ra);
		rs->areas[i].size = ra.size;
		void *p = mmap(NULL, ra.size, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, ra.offset);
		if (p == MAP_FAILED) {
			rv = errno;
			goto fail;
		}
		rs->areas[i].start = p;
	}
	return 0;
 fail:
	for (; i>= 0; --i) {
		if (rs->areas[i].start != NULL)
			munmap(rs->areas[i].start, rs->areas[i].size);
	}
	return rv;
}

static void unmap_reg_area_set(struct reg_area_set *rs)
{
	for (int i = 0; i < rs->num_areas; ++i)
		munmap(rs->areas[i].start, rs->areas[i].size);
}

static void fini_reg_area_set(struct reg_area_set *rs)
{
	free(rs->areas);
}

/**
 * @brief mmap core user register
 *
 * @param[in] handle VEDL handler
 * @param core_id Physical VE core ID
 *
 * @return non-null handle on success. NULL on failure.
 */
vedl_user_reg_handle *vedl_mmap_usr_reg(vedl_handle *handle, int core_id)
{
	int err;
	vedl_user_reg_handle *rv;
	rv = malloc(sizeof(*rv));
	if (rv == NULL)
		return rv;

	int num_areas = handle->arch_class->num_user_regs_area;
	err = init_reg_area_set(&rv->user_regs, num_areas);
	if (err != 0)
		goto err_init_reg_area_set;

	err = map_core_reg_area_set(handle->vefd, core_id, &rv->user_regs,
				handle->arch_class->get_user_regs_area);
	if (err != 0)
		goto err_map_reg_area_set;

	rv->get_usr_offset = handle->arch_class->get_usr_offset;
	return rv;

 err_map_reg_area_set:
	fini_reg_area_set(&rv->user_regs);
 err_init_reg_area_set:
	free(rv);
	errno = err;
	return NULL;
}

/**
 * @brief mmap core system register
 *
 * @param[in] handle VEDL handle
 * @param core_id Physical VE core ID
 *
 * @return non-null handle on success. NULL on failure.
 */
vedl_sys_reg_handle *vedl_mmap_sys_reg(vedl_handle *handle, int core_id)
{
	int err;
	vedl_sys_reg_handle *rv;
	rv = malloc(sizeof(*rv));
	if (rv == NULL)
		return rv;

	int num_areas = handle->arch_class->num_sys_regs_area;
	err = init_reg_area_set(&rv->sys_regs, num_areas);
	if (err != 0)
		goto err_init_reg_area_set;

	err = map_core_reg_area_set(handle->vefd, core_id, &rv->sys_regs,
				handle->arch_class->get_sys_regs_area);
	if (err != 0)
		goto err_map_reg_area_set;

	rv->get_sys_offset = handle->arch_class->get_sys_offset;
	return rv;

 err_map_reg_area_set:
	fini_reg_area_set(&rv->sys_regs);
 err_init_reg_area_set:
	free(rv);
	errno = err;
	return NULL;
}

/**
 * @brief mmap system common register
 *
 * @param[in] handle VE handle
 *
 * @return non-null handle on success. NULL on failure.
 */
vedl_common_reg_handle *vedl_mmap_cnt_reg(vedl_handle *handle)
{
	int err, i;
	vedl_common_reg_handle *rv;
	rv = malloc(sizeof(*rv));
	if (rv == NULL)
		return rv;

	int num_areas = handle->arch_class->num_common_regs_area;
	err = init_reg_area_set(&rv->common_regs, num_areas);
	if (err != 0) {
		goto err_init_reg_area_set;
	}
	for (i = 0; i < num_areas; ++i) {
		struct reg_area ra;
		rv->common_regs.areas[i].start = MAP_FAILED;
		err = handle->arch_class->get_common_regs_area(i, &ra);
		rv->common_regs.areas[i].size = ra.size;
		void *p = mmap(NULL, ra.size, PROT_READ | PROT_WRITE,
				MAP_SHARED, handle->vefd, ra.offset);
		if (p == MAP_FAILED) {
			err = errno;
			goto err_map;
		}
		rv->common_regs.areas[i].start = p;
	}
	rv->arch_class = handle->arch_class;
	return rv;

 err_map:
	for (; i>= 0; --i) {
		if (rv->common_regs.areas[i].start != NULL)
			munmap(rv->common_regs.areas[i].start,
				rv->common_regs.areas[i].size);
	}

	fini_reg_area_set(&rv->common_regs);
 err_init_reg_area_set:
	free(rv);
	errno = err;
	return NULL;
}

/**
 * @brief unmap core user register
 *
 * @param user_reg_handle core user register handle
 *
 * @return zero on success; non-zero on failure.
 *
 */
int vedl_munmap_usr_reg(vedl_user_reg_handle *user_reg_handle)
{
	unmap_reg_area_set(&user_reg_handle->user_regs);
	fini_reg_area_set(&user_reg_handle->user_regs);
	free(user_reg_handle);
	return 0;
}

/**
 * @brief unmap core system register
 *
 * @param sys_reg_handle core system register handle
 *
 * @return zero on success; non-zero on failure.
 *
 */
int vedl_munmap_sys_reg(vedl_sys_reg_handle *sys_reg_handle)
{
	unmap_reg_area_set(&sys_reg_handle->sys_regs);
	fini_reg_area_set(&sys_reg_handle->sys_regs);
	free(sys_reg_handle);
	return 0;
}

/**
 * @brief unmap system common register
 *
 * @param common_reg_handle system common register handle
 *
 * @return zero on success; non-zero on failure.
 *
 */
int vedl_munmap_common_reg(vedl_common_reg_handle *common_reg_handle)
{
	unmap_reg_area_set(&common_reg_handle->common_regs);
	fini_reg_area_set(&common_reg_handle->common_regs);
	free(common_reg_handle);
	return 0;
}

/**
 * @brief Unmap MMIO mmap
 * @details This function will call unmap_mapping_range() to rip off
 *         the mapped page frame by VE Driver.
 *
 * @param[in] handle VEDL handle
 * @param offset offset address from top of MMIO area
 * @param size size to unmap
 *
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           EINVAL  Invalid offset, invalid size
 */
int vedl_unmap_mmio(vedl_handle *handle, off_t offset, size_t size)
{
	int retval = -1;
	struct ve_unmap arg;

	arg.offset = offset;
	arg.size = size;

	retval = ioctl(handle->vefd, VEDRV_CMD_UNMAP, &arg);

	return retval;
}

/**
 * @brief Get host pid
 * @details This function get the host pid which is converted
 * 	   from namespace_pid. First, find name space from
 * 	   host_pid. After that, find task from name space
 * 	   and namespace_pid. Finaly, get the host pid which
 * 	   is converted from namespace_pid.
 *
 * @param[in] handle VEDL handle
 * @param[in] host_pid The host pid in the host to identify the namespace.
 * @param[in] namespace_pid The namespace pid to be converted.
 *
 * @return The host pid.
 */
pid_t vedl_host_pid(vedl_handle *handle, pid_t host_pid, pid_t namespace_pid)
{
	struct ve_get_host_pid arg;
	
	arg.host_pid = host_pid;
	arg.namespace_pid = namespace_pid;

	return ioctl(handle->vefd, VEDRV_CMD_HOST_PID, &arg);
}

/**
 * @brief Get file descriptor
 * @details This function returns a file descriptor of VE device
 *         bound to VEDL handle specified by the argument.
 *
 * @param[in] handle VEDL handle
 *
 * @return File descriptor of VE device.
 */
int vedl_get_fd(const vedl_handle *handle)
{
	return handle->vefd;
}

/**
 * @brief Reset VE Chip
 * 
 * @details This function assert VE chip reset by using
 *         secondary bus reset or original reset method.
 *         function level reset only for Aurora3
 *         After success, this function will update VE firmware.
 *         Please note that AER is disabled during reset.
 *
 * @param[in] handle VEDL handle
 * @param sbr set 0 to use original reset method.
 *            set 1 to assert secondary bus reset.
 *            set 2 to function level reset
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           EAGAIN on invalid VE state
 *           ENOMEM on lack of memory
 *           EINVAL on invalid argument
 *           EIO on failure of firmware update
 */
int vedl_reset_ve_chip(vedl_handle *handle, int sbr)
{
	return ioctl(handle->vefd, VEDRV_CMD_VE_VE_RESET, sbr);
}

/**
 * @brief Update VE firmware only Aurora1
 *
 * @details This function will update VE firmware manually.
 *         Usually it's done automatically at the time of driver
 *         initialization.
 *         Please note that AER is disabled during updating.
 *         for compatibilities with old libved
 *
 * @param[in] handle VEDL handle
 *
 * @return 0 on success.
 *         Negative on failure.
 *         errno:
 *           EAGAIN on invalid VE state
 *           ENOMEM on lack of memory
 *           EINVAL on invalid argument
 *           EIO on failure of firmware update
 */
int vedl_update_firmware(vedl_handle *handle)
{
	if (handle->arch_class->update_firmware)
	  return handle->arch_class->update_firmware(handle);

	errno = ENOTSUP;
	return -1;
}

/**
 * @brief wait for interuption only Aurora1
 *
 * @details
 * This function blocks until specified MSI-X entry interrupt occur.
 * for compatibilities with old libved
 *
 * @param[in] handle VEDL handler
 * @param entry MSI-X entry number to wait
 * @param[in] timeout timeout count
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           ETIMEDOUT time expired.
 *           EFAULT    invalid argument.
 *           EINTR     receiving signal.
 */

/*
 * It can specify timeout.tv_sec lower MAX_WAIT_INTERRUPT
 *
 * #define HZ 1000                              refer param.h
 * #define MAX_JIFFY_OFFSET ((LONG_MAX >> 1)-1) refer jiffies.h
 * #define MAX_WAIT_INTERRUPT (MAX_JIFFY_OFFSET / HZ)
 *   MAX_WAIT_INTERRUPT .==. 2147482
*/
#define MAX_WAIT_INTERRUPT 10

int vedl_wait_interrupt(vedl_handle *handle, int entry,
			   struct timespec *timeout)
{
	int ret;
	struct timespec least_timeout, max_timeout;
	uint64_t utv_sec;

	if (handle->arch_class->wait_interrupt == NULL){
		errno = ENOTSUP;
		return -1;
	}

	if(timeout->tv_sec <= (uint64_t)(MAX_WAIT_INTERRUPT)) {
		return handle->arch_class->wait_interrupt(handle, entry, timeout);
	} else {
		least_timeout.tv_sec = timeout->tv_sec;
		least_timeout.tv_nsec = timeout->tv_nsec;
		max_timeout.tv_sec = (time_t)(MAX_WAIT_INTERRUPT);
		max_timeout.tv_nsec = (time_t)(0);
		utv_sec =   (uint64_t)(timeout->tv_sec);
		while (utv_sec > 0) {
			ret = handle->arch_class->wait_interrupt(handle,
							entry, &max_timeout);
			if(ret == 0){
				return ret;
			}
			if(errno != ETIMEDOUT) {
				return ret;
			}
			errno = 0;
			utv_sec -= MAX_WAIT_INTERRUPT;
			if(utv_sec < MAX_WAIT_INTERRUPT) {
				break;
			}
		}
		if(utv_sec > 0) {
			errno = 0;
			least_timeout.tv_sec = (time_t)(utv_sec);
			ret = handle->arch_class->wait_interrupt(handle,
							entry, &least_timeout);
		}
		return ret;
	}
}

/**
 * @brief Clear EXSRAR target memory of core
 *
 * @details
 * This function clear of EXSRAR of core
 *
 * @param[in] handle VEDL handle
 * @param core_id Physical VE Core ID
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           EPERM   The caller is not permitted
 *	     EINVAL  Invalid core_id.
 */
int vedl_reset_exsrar_mem(vedl_handle *handle, int core_id)
{
	return ioctl(handle->vefd, VEDRV_CMD_RST_EXSRAR_MEM, core_id);
}


/**
 * @brief Notify fault to driver
 *
 * @details
 * This function Notify Fault of VEOS to VE driver
 * VE driver kill VEOS
 *
 * @param[in] handle VEDL handle
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_notify_fault(vedl_handle *handle) 
{
  if (handle->arch_class->notify_fault)
    return handle->arch_class->notify_fault(handle);

  errno = ENOTSUP;
  return -1;

}


/**
 * @brief Request ownership of VE driver
 *
 * @details
 * This function request ownership of VE driver
 *
 * @param[in] handle VEDL handle
 * @param[in] timeout timeout value
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_request_ownership(vedl_handle *handle, int timeout)
{

  if (handle->arch_class->request_ownership)
    return handle->arch_class->request_ownership(handle, timeout);

  errno = ENOTSUP;
  return -1;

}

/**
 * @brief Release ownership of VE driver
 *
 * @details
 * This function release  ownership of VE driver
 *
 * @param[in] handle VEDL handle
 * @param[in] timeout timeout value
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_release_ownership(vedl_handle *handle)
{

  if (handle->arch_class->release_ownership)
    return handle->arch_class->release_ownership(handle);

  errno = ENOTSUP;
  return -1;

}

/**
 * @brief get EXS Resgier
 *
 * @details
 * This function get EXS Register
 *
 * @param[in] handle: VEDL handle
 *
 * @param[out] exs  : Pointing to the EXS register value
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           ENOSUP: not support
 */

int vedl_get_exs_register(vedl_handle *handle,ve_reg_t *exs)
{

  if (handle->arch_class->get_exs_register)
    return handle->arch_class->get_exs_register(handle, exs);

  errno = ENOTSUP;
  return -1;

}
/**
 * @brief complete memory clear
 *
 * @details
 * This function  complete memory clear
 *
 * @param[in] handle VEDL handle
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_complete_memclear(vedl_handle *handle)
{
  if (handle->arch_class->complete_memclear)
    return handle->arch_class->complete_memclear(handle);

  errno = ENOTSUP;
  return -1;

}

/**
 * @brief handle clock gating
 *
 * @details
 * This function  handle clock gating
 *
 * @param[in] handle VEDL handle
 * @param[flag] clock state (1:on, 0:off)
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_handle_clock_gating(vedl_handle *handle, int flags)
{
  if (handle->arch_class->handle_clock_gating)
    return handle->arch_class->handle_clock_gating(handle, flags);

  errno = ENOTSUP;
  return -1;

}

/**
 * @brief get clock gating state
 *
 * @details
 * This function  get clock gating state
 *
 * @param[in] handle VEDL handle
 * @param[out] state  : Pointing to clock gating state value
 *
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_get_clock_gating_state(vedl_handle *handle, uint64_t *state)
{
  if (handle->arch_class->get_clock_gating_state)
    return handle->arch_class->get_clock_gating_state(handle, state);

  errno = ENOTSUP;
  return -1;

}

//@}
