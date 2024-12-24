# #!/usr/bin/env bash
# INSTALL_DIR=${PWD}
# if [ $1 ]; then
#   export LOCAL_PKG_PATH=$1
#   echo "defined LOCAL_PKG_PATH: $1"
# fi
# CURDIR=$(pwd)
# SUBDIRNAME="build_x86"

# ##############################################
# # read meta file and to confirm the dependency.
# automsg_version=`python3 cicd/version_get.py automsg_version`
# philog_version=`python3 cicd/version_get.py philog_version`


# echo "[META][INFO]: auto msg      : $automsg_version"
# echo "[META][INFO]: philog        : $philog_version"

# ##############################################

# cmake \
#   -DARCH_PLAT_VENDOR=x86_64-ubuntu-linux-gcc9.3.0 \
#   -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
#   -DCMAKE_BUILD_TYPE=RelWithDebugInfo \
#   -DENABLE_ASAN=OFF \
#   -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
#   ..

# make -j15 && make install


#!/bin/bash

# 检查是否提供了必要的参数
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <build_type> <source_directory>"
    exit 1
fi

# 获取传递的参数
BUILD_TYPE=$1
SOURCE_DIR=$2

# 检查源代码目录是否存在
if [ ! -d "$SOURCE_DIR" ]; then
    echo "Error: Source directory does not exist."
    exit 1
fi

# 设置构建目录和安装目录
BUILD_DIR="$SOURCE_DIR/build"
INSTALL_DIR="./install"

# 创建并进入构建目录
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR" || exit

# 配置 CMake 项目，指定安装路径为当前目录下的 install
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" ..

# 编译项目
cmake --build . --config $BUILD_TYPE

# 安装项目到指定的安装目录
cmake --build . --target install --config $BUILD_TYPE

# 返回到初始目录
cd - || exit

echo "Installation completed to $INSTALL_DIR"
