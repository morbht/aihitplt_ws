#!/bin/bash

# 获取脚本所在目录（工作空间根目录）
WS_PATH=$(cd "$(dirname "$0")"; pwd)


cd $WS_PATH

# 1. 先编译消息包
echo ""
echo "第一步：编译消息包 aihitplt_msgs..."
catkin_make --only-pkg-with-deps aihitplt_msgs

if [ $? -ne 0 ]; then
    echo "错误：消息包编译失败！"
    exit 1
fi
echo "消息包编译完成！"

# 2. 删除 build 文件夹清除 whitelist 限制
echo ""
echo "第二步：清除 whitelist 限制..."
rm -rf build/
echo "build 文件夹已删除"

# 3. 重新编译所有包
echo ""
echo "第三步：编译所有功能包..."
catkin_make

if [ $? -ne 0 ]; then
    echo "错误：编译失败！"
    exit 1
fi

echo ""
echo "=========================================="
echo "编译完成！"
echo "=========================================="
echo ""
echo "运行以下命令生效："
echo "source ~/.bashrc"
