#!/bin/sh
set -e

./autogen.sh
./configure --prefix=/opt/nec/ve/veos/ --libdir=/opt/nec/ve/veos/lib64 --with-release-id=`date +%Y%m%d%H%M`
make rpm
