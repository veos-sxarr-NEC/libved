#
# VE Driver Library
#
# Copyright (C) 2017-2018 NEC Corporation
# This file is part of the VE Driver Library.
#
# The VE Driver Library is free software; you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either version
# 2.1 of the License, or (at your option) any later version.
#
# The VE Driver Library is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with the VE Driver Library; if not, see
# <http://www.gnu.org/licenses/>.
#
lib_LTLIBRARIES = libved.la
dist_noinst_DATA = libved.map
libved_la_SOURCES = libved.c libved_ve1.c libved_ve3.c \
                    register.h register.c register_ve1.c register_ve3.c \
                    internal.h internal_ve1.h internal_ve3.h
libved_la_CFLAGS = -std=gnu99 -O2
libved_la_LDFLAGS = -version-info 3:0:0 \
                   -Wl,--version-script=$(srcdir)/libved.map
include_HEADERS = libved.h libved_ve1.h libved_ve3.h \
                  ve_hw.h ve_hw_ve1.h ve_hw_ve3.h
