#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([libved], [1.3.1], [t-aoyama@ap.jp.nec.com])
AC_CONFIG_SRCDIR([src/libved.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_AUX_DIR([./build-aux])

AC_SUBST(RELEASE)

AM_INIT_AUTOMAKE([foreign -Wall -Werror])
AM_PROG_AR
LT_INIT
AC_CONFIG_MACRO_DIR([m4])

AC_ARG_WITH(veos, [AS_HELP_STRING([--with-veos],
	[VEOS installed prefix])],
	[with_veos=$withval], [with_veos=/opt/nec/ve/veos])
AC_ARG_WITH([release-id], [AS_HELP_STRING([--with-release-id],
            [Specify release id])],
            [RELEASE=${with_release_id}],
            [RELEASE="1"])
AC_ARG_WITH([no-memcpy], [AS_HELP_STRING([--with-no-memcpy],
	[Without memcpy option])],
	[NO_MEMCPY=${with_no_memcpy}],
	[NO_MEMCPY="0"])
AS_IF([test "$NO_MEMCPY" != 0],
	[CFLAGS=$CFLAGS" -DNO_MEMCPY"])
AC_SUBST(NO_MEMCPY)

CPPFLAGS=$CPPFLAGS" -I${with_veos}/include"
# Checks for programs.
AC_PROG_CC

# Checks for libraries.
# FIXME: Replace `main' with a function in `-lpthread':
AC_CHECK_LIB([pthread], [main])
# FIXME: Replace `main' with a function in `-ludev':
AC_CHECK_LIB([udev], [main])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h limits.h stddef.h stdlib.h string.h sys/ioctl.h unistd.h])
AC_CHECK_HEADERS([ve_drv.h vp.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_INLINE
AC_TYPE_OFF_T
AC_TYPE_PID_T
AC_TYPE_SIZE_T
AC_CHECK_MEMBERS([struct stat.st_rdev])
AC_TYPE_UID_T
AC_TYPE_UINT64_T

# Checks for library functions.
AC_ENABLE_SHARED
AC_DISABLE_STATIC
AC_FUNC_MALLOC
AC_FUNC_MMAP
AC_CHECK_FUNCS([strerror strtol strtoul strtoull])

AC_CONFIG_FILES([Makefile
		 libved.spec
		 src/Makefile
		 debian/rules
		 debian/control
		 debian/libved.install
		 debian/libved-dev.install])
AC_OUTPUT
