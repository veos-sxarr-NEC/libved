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
 * @file  libved.c
 * @brief VE driver library
 */

#define _GNU_SOURCE
#include <unistd.h>
#include <stdlib.h>
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

	return handle;

 abi_err:
	udev_device_unref(handle->udev);
 udev_err:
 fstat_err:
	close(handle->vefd);
 open_err:
	free(handle);
 malloc_err:
	errno = errsv;
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
	int ret;
	int errsv = 0;

	new_handle = (vedl_handle *)malloc(sizeof(vedl_handle));
	if (new_handle == NULL)
		return NULL;

	/* copy handle */
	memcpy(new_handle, p_handle, sizeof(vedl_handle));
	udev_device_ref(new_handle->udev);

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
	udev_device_unref(new_handle->udev);
	free(new_handle);
	errno = errsv;
	return NULL;
}

/**
 * @brief Close VEDL handle
 *
 * @param[in] VEDL handle
 *
 * @return 0 on success.
 *         -1 on failure.
 */
int vedl_close_ve(vedl_handle *handle)
{
	int ret;

	ret = close(handle->vefd);
	if (ret)
		goto err;
	udev_device_unref(handle->udev);
	free(handle);
 err:
	return ret;
}

/**
 * @brief get syscall arguments
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
int vedl_wait_exception(vedl_handle *handle, reg_t *exs)
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
 * @param pid_t PID to translate
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
 * @brief return physical address of process virtual address for MMM, T&D
 *
 * @param[in] handle VEDL handle
 * @param[in] addr virtual address to translate
 * @param pid_t PID to translate
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
	readlen = read(fd, str, length);
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
	retval = close(fd);

	return retval;
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
	int retval;

	switch (bar) {
	case 0:
		retval = vedl_read_from_sysfs(handle, "bar0_size", string,
				PATH_MAX);
		break;
	default:
		return -EINVAL;
	}
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
	int retval;

	switch (bar) {
	case 0:
		retval = vedl_read_from_sysfs(handle, "bar0_addr", string,
				PATH_MAX);
		break;
	case 3:
		retval = vedl_read_from_sysfs(handle, "bar3_addr", string,
				PATH_MAX);
		break;
	default:
		return -EINVAL;
	}
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
 * @brief wait for interuption
 *
 * @detail
 * This function blocks until specified MSI-X entry interrupt occur.
 *
 * @param[in] handle VEDL handler
 * @param msix_entry MSI-X entry number to wait
 * @param[in] timeout timeout count
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           ETIMEDOUT time expired.
 *	     EFAULT    invalid argument.
 *	     EINTR     receiving signal.
 */
int vedl_wait_interrupt(vedl_handle *handle, msix_entry_t entry,
			struct timespec *timeout)
{
	int retval = 0;
	struct ve_wait_irq_arg arg;

	arg.bits.lower = 0;
	arg.bits.upper = 0;
	arg.timeout = NULL;

	if (entry < 64)
		arg.bits.lower = 0x1ULL << entry;
	else
		arg.bits.upper = 0x1ULL << (entry - 64);
	arg.timeout = timeout;

	retval = ioctl(handle->vefd, VEDRV_CMD_WAIT_INTR, &arg);
	if (retval > 0)
		return 0;

	return retval;
}

/**
 * @brief mmap core user register
 *
 * @param[in] handle VEDL handler
 * @param core_id Physical VE core ID
 *
 * @return 0 on success. MAP_FAILED on failure.
 */
core_user_reg_t *vedl_mmap_usr_reg(vedl_handle *handle, int core_id)
{
	return mmap(NULL, sizeof(core_user_reg_t),
			PROT_READ | PROT_WRITE, MAP_SHARED,
			handle->vefd, VEDRV_MAP_BAR2_OFFSET +
			PCI_BAR2_CREG_SIZE * core_id);
}

/**
 * @brief mmap core system register
 *
 * @param[in] handle VEDL handle
 * @param core_id Physical VE core ID
 *
 * @return 0 on success. MAP_FAILED on failure.
 */
core_system_reg_t *vedl_mmap_sys_reg(vedl_handle *handle, int core_id)
{
	return mmap(NULL, sizeof(core_system_reg_t),
			PROT_READ | PROT_WRITE, MAP_SHARED,
			handle->vefd, VEDRV_MAP_BAR2_OFFSET +
			PCI_BAR2_CREG_SIZE * core_id + sizeof(core_user_reg_t));
}

/**
 * @brief mmap system common register
 *
 * @param[in] handle VE handle
 *
 * @return 0 on success. MAP_FAILED on failure.
 */
system_common_reg_t *vedl_mmap_cnt_reg(vedl_handle *handle)
{
	return mmap(NULL, sizeof(system_common_reg_t),
			PROT_READ | PROT_WRITE, MAP_SHARED,
			handle->vefd, VEDRV_MAP_BAR2_OFFSET +
			PCI_BAR2_SCR_OFFSET);
}

/**
 * @brief Unmap MMIO mmap
 * @detail This function will call unmap_mapping_range() to rip off
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
 * @brief Update VE firmware
 * @detail This function will update VE firmware manually.
 *         Usually it's done automatically at the time of driver
 *         initialization.
 *         Please note that AER is disabled during updating.
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
	return ioctl(handle->vefd, VEDRV_CMD_UPDATE_FIRMWARE);
}

/**
 * @brief Reset VE Chip
 * @detail This function assert VE chip reset by using
 *         secondary bus reset or original reset method.
 *         After success, this function will update VE firmware.
 *         Please note that AER is disabled during reset.
 *
 * @param[in] handle VEDL handle
 * @param sbr set 1 to assert secondary bus reset.
 *            set 0 to use original reset method.
 *
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
	return ioctl(handle->vefd, VEDRV_CMD_VE_RESET, sbr);
}
