#!/usr/bin/env bash
INSTALL_DIR=${PWD}
if [ $1 ]; then
  export LOCAL_PKG_PATH=$1
  echo "defined LOCAL_PKG_PATH: $1"
fi
CURDIR=$(pwd)
SUBDIRNAME="build_x86"

##############################################
# read meta file and to confirm the dependency.
automsg_version=`python3 cicd/version_get.py automsg_version`
philog_version=`python3 cicd/version_get.py philog_version`


echo "[META][INFO]: auto msg      : $automsg_version"
echo "[META][INFO]: philog        : $philog_version"

##############################################

cmake \
  -DARCH_PLAT_VENDOR=x86_64-ubuntu-linux-gcc9.3.0 \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
  -DCMAKE_BUILD_TYPE=RelWithDebugInfo \
  -DENABLE_ASAN=OFF \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  ..

make -j15 && make install
