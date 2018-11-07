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

AUTOMAKE_OPTIONS = foreign
ACLOCAL_AMFLAGS = -I m4
SUBDIRS = src
dist_noinst_DATA = libved.spec
EXTRA_DIST = debian

rpm: dist
	mkdir -p $(distdir)/rpmbuild/{BUILD,RPMS,SOURCES,SRPMS,SPECS}
	rpmbuild -ta --clean --define='_topdir $(CURDIR)/$(distdir)/rpmbuild' ${distdir}.tar.gz
