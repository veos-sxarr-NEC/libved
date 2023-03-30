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
 * @file  libved_ve3.c
 * @brief VE3 specific part of VE driver library
 */

#define _GNU_SOURCE
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <limits.h>
#include <sys/ioctl.h>
#define _VE_ARCH_VE3_ (1)
#include <ve_drv.h>
#include "libved.h"
#include "ve_hw.h"
#include "internal.h"

#define CHECK_VE3(hdl) \
	do { if (!IS_VE3(hdl)) { \
		errno = ENOTSUP; \
		return -1; \
	} } while (0)

static int ve3_interrupt_entry_to_intvec_bit(vedl_interrupt_entry_t entry)
{
	if (0 <= entry && entry <= VE_INTR_CORE31)
		return entry;

	if (VE_INTR_UDMA00 <= entry && entry <= VE_INTR_UDMA63)
		return entry - VE_INTR_CORE00_HDMA + 128;

	if (VE_INTR_CORE00_FAULT <= entry && entry <= VE_INTR_CORE31_FAULT)
		return entry - VE_INTR_CORE00_FAULT + 192;

	switch (entry) {
	case VE_INTR_P_DMA:
		return 64;
	case VE_INTR_PCI_ACCESS_EXC:
		return 96;
	/* TODO: add interrupt types regarding faults or warnings */
	default:
		return -1;
	}
}

static int ve3_set_wait_irq(struct ve3_wait_irq *cond, int intvec_bit)
{
	cond->ve_wait_irq_type = VEDRV_IRQ_TYPE_VE3;
	memset(cond->intvec, 0, sizeof(cond->intvec));

	if (intvec_bit < 0 || 256 <= intvec_bit)
		return EINVAL;
	cond->intvec[intvec_bit / 64] = 1UL << (intvec_bit % 64);
	return 0;
}

static int ve3_create_wait_irq(vedl_interrupt_entry_t entry,
				struct ve_wait_irq **irq)
{
	int rv, intvec_bit;
	*irq = malloc(sizeof(struct ve3_wait_irq));
	if (*irq == NULL)
		return errno;
	intvec_bit = ve3_interrupt_entry_to_intvec_bit(entry);
	rv = ve3_set_wait_irq((struct ve3_wait_irq *)*irq, intvec_bit);
	if (rv != 0) {
		free(*irq);
		*irq = NULL;
	}
	return rv;
}

/* for compatibilities with old libved */
int vedl_ve3_wait_interrupt(vedl_handle *handle, int msix_entry,
                        struct timespec *timeout)
{
	struct ve3_wait_irq *cond;
	CHECK_VE3(handle);
	int ientry;
	int intvec_bit;
	int ret;

	// only suport 35
	if (msix_entry != 35 ){
		errno = EINVAL;
		return -1;
	}

	cond = malloc(sizeof(struct ve3_wait_irq));
	if (cond == NULL)
		return errno;

	cond->ve_wait_irq_type = VEDRV_IRQ_TYPE_VE3;
	memset(cond->intvec, 0, sizeof(cond->intvec));

	// s/w 0x00001FF7ffffffff
	// h/w 0xffffffffeff80000

	for (ientry = VE_INTR_CORE00_FAULT; ientry<VE_INTR_MAX; ientry++){
		intvec_bit = ve3_interrupt_entry_to_intvec_bit( ientry );
		if(intvec_bit != -1){
			cond->intvec[intvec_bit / 64] |= (1UL << (intvec_bit % 64));
		}
	}
	intvec_bit = ve3_interrupt_entry_to_intvec_bit( VE_INTR_HMC_ERR );
	if(intvec_bit != -1) {
		cond->intvec[intvec_bit / 64] &= ~(intvec_bit % 64);
	}

	// expect: cond->intvec[3] = 0x00001FF7ffffffff
	cond->intvec[3] |= 0x00001FF700000000;

	ret = vedl__wait_for_interrupt(handle, (struct ve_wait_irq *)cond,
						timeout);
	free(cond);

	return ret;
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
int vedl_ve3_notify_fault(vedl_handle *handle) 
{
  int retval = -1;
  retval = ioctl(handle->vefd, VEDRV_CMD_NOTIFY_FAULT);
  if ( retval < 0 )
    fprintf(stderr, "%s notify_fault\n", strerror(errno));
  return retval;
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
 *         negative value on failure.
 *         errno:
 *           EBUSY  : any one has onwership
 *           ESRCH  : owner process does not exist
 *           ETIME  : timeout
 */
int vedl_ve3_request_ownership(vedl_handle *handle, int timeout)
{

  int retval = -1;
  int ownership_fd, rv, retn;
  char dummy_buf[16];
  char attrData[32];
  struct pollfd ufds[1];


  // real code 
  char sysfsname[PATH_MAX];
  retval = snprintf(sysfsname, PATH_MAX, "%s/ownership", vedl_get_sysfs_path(handle));
  if (retval < 0)
    return retval;


  if ((ownership_fd = open(sysfsname, O_RDONLY)) < 0)
  {
    fprintf(stderr, "%s :request_ownership open sysfs \n", strerror(errno));
    retval = -1;
    goto error_1;
  }


  ufds[0].fd = ownership_fd;
  ufds[0].events = POLLPRI|POLLERR;
  ufds[0].revents = 0;

  while(1){

    /*
     * arg 1: request
     */
    retval = ioctl(handle->vefd, VEDRV_CMD_OWNERSHIP, (int)1);

    if (retval < 0 ) {
      if (errno != EBUSY){
	fprintf(stderr, "%s :request_ownership write\n", strerror(errno));
	goto error_2;
      }
      //fprintf("any one has ownership :%s %d %d\n",sysfsname, errno,retval);
      // Someone suggested dummy reads before the poll() call
      if (-1 == read(ownership_fd, dummy_buf, 10)) { /* Trac# 383 */
	fprintf(stderr, "%s :request_ownership read \n", strerror(errno));
	goto error_2;
      }
      if (( rv = poll( ufds, 1, timeout)) < 0 )	{
	  fprintf(stderr, "%s :request_ownership poll \n", strerror(errno));
	  goto error_2;

      } else if (rv == 0) {
	  /*
	   * whe timeout, errno not set and still EBUSY at write()
	   * so, fix.
	   */
	  errno = ETIME;
	  fprintf(stderr, "%s :request_ownership timeout\n", strerror(errno));
	  goto error_2;
      } else if (ufds[0].revents & (POLLPRI|POLLERR)) {
	  lseek(ownership_fd, 0L, SEEK_SET);
	  // discard, not use
	  retn = read( ownership_fd, attrData, 32 );
	  // disable warning:set but not used [-Wunused-but-set-variable]
	  // retval is reset to 0 at end.
	  retval = retn;
	  //fprintf(stdout, "request_ownership wakeup%s: %s\n", sysfsname, attrData );
	  lseek(ownership_fd, 0L, SEEK_SET);
      }

    }else{
      // fprintf(stdout, "request_ownership write to %s %s len=%d\n",sysfsname, "1", retval);
      break;
    }

  }
  retval = 0;
  // fprintf(stdout, "request_ownership revents[0]: %08X\n", ufds[0].revents );
error_2:
  close( ownership_fd );
error_1:
  return retval;
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
 *         negative value on failure.
 *         errno:
 *           EACCES : not owner
 *           ESRCH  : owner process does not exist
 *           EINVAL : no one has ownership
 */
int vedl_ve3_release_ownership(vedl_handle *handle)
{

  int retval = -1;

  /*
   * arg 0: request
   */

  retval = ioctl(handle->vefd, VEDRV_CMD_OWNERSHIP, (int)0);
  if ( retval < 0 )
    fprintf(stderr, "%s release_ownership\n", strerror(errno));
  return retval;

}

int vedl_ve3_get_pci_bar4_crarea_address(vedl_handle *handle,
						uint64_t *addr)
{
        int rv;
        uint64_t bar4_addr;
        rv = vedl_get_pci_bar_address(handle, &bar4_addr, 4);
        *addr = bar4_addr + VEDRV_VE3_PCI_BAR4_SCR_OFFSET +
		offsetof(ve3_system_common_reg_nonpre_t, CR);
        return rv;
}

/**
 * @brief Read  EXS register
 *
 * @details
 * This function clear of EXSRAR of core
 *
 * @param[in] handle: VEDL handle
 * @param[out] exs  : Pointing to the EXS register value
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *
 *           EINVAL  Invalid core_id.
 */
int vedl_ve3_get_exs_register(vedl_handle *handle,  ve_reg_t *exs)
{
        return ioctl(handle->vefd, VEDRV_CMD_GET_EXS_REG, exs);
}

/**
 * @brief complete memory clear
 *
 * @details
 * This function  complete memory initialize
 *
 * @param[in] handle VEDL handle
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_ve3_complete_memclear(vedl_handle *handle)
{
  int retval = -1;
  retval = ioctl(handle->vefd, VEDRV_CMD_COMPLETE_MEMCLEAR);
  if ( retval < 0 )
    fprintf(stderr, "%s complete_memclear\n", strerror(errno));
  return retval;
}

/**
 * @brief handle clock gating
 *
 * @details
 * This function handle clock gating
 *
 * @param[in] handle VEDL handle
 * @param[flag] clock state (1:on, 0:off)
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_ve3_handle_clock_gating(vedl_handle *handle, int flags)
{
  int retval = -1;
  retval = ioctl(handle->vefd, VEDRV_CMD_HANDLE_CLOCK_GATING, flags);
  if ( retval < 0 )
    fprintf(stderr, "%s handle_clock_gating\n", strerror(errno));
  return retval;
}

/**
 * @brief get clock gating state
 *
 * @details
 * This function get clock gating state
 *
 * @param[in] handle VEDL handle
 * @param[out] state  : Pointing to clock gating state value
 *
 * @return 0 on success. Negative on failure.
 *         errno:
 *           XXX
 */
int vedl_ve3_get_clock_gating_state(vedl_handle *handle, uint64_t *state)
{
  int retval = -1;
  retval = ioctl(handle->vefd, VEDRV_CMD_GET_CLOCK_GATING_STATE, state);
  if ( retval < 0 )
    fprintf(stderr, "%s get_clock_gating_state\n", strerror(errno));
  return retval;
}

const struct vedl_arch_class _vedl_ve3_arch_class = {
	.name = VE_DRV_ARCH_NAME_VE3,
	/* BAR01 (bar0_addr and bar0_size) and BAR4 (bar4_addr and bar4_size) */
	.bars_map = (1 << 0) | (1 << 4),
	.num_user_regs_area = 2,
	.get_user_regs_area = _vedl_ve3_get_user_regs_area,
	.get_usr_offset = _vedl_ve3_get_usr_offset,
	.num_sys_regs_area = 2,
	.get_sys_regs_area = _vedl_ve3_get_sys_regs_area,
	.get_sys_offset = _vedl_ve3_get_sys_offset,
	.num_common_regs_area = 2,
	.get_common_regs_area = _vedl_ve3_get_common_regs_area,
	.create_wait_irq = ve3_create_wait_irq,
	.update_firmware = NULL,
	.wait_interrupt = vedl_ve3_wait_interrupt,
	.request_ownership = vedl_ve3_request_ownership,
	.release_ownership = vedl_ve3_release_ownership,
	.notify_fault = vedl_ve3_notify_fault,
	.get_exs_register = vedl_ve3_get_exs_register,
        .complete_memclear = vedl_ve3_complete_memclear,
        .handle_clock_gating = vedl_ve3_handle_clock_gating,
        .get_clock_gating_state = vedl_ve3_get_clock_gating_state,
};
VEDL_ARCH_CLASS(_vedl_ve3_arch_class);
