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
#define __VE_HW_H


#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef uint64_t ve_reg_t;

#if defined(_VE_ARCH_VE3_)
#include <ve_hw_ve3.h>
#elif defined(_VE_ARCH_VE1_)
#include <ve_hw_ve1.h>
#else
#error "specify _VE_ARCH_VE3_ or _VE_ARCH_VE1_"
#endif

#if defined(__cplusplus)
}
#endif
#endif
