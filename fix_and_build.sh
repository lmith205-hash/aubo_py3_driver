#!/bin/bash

echo ">>> 开始自动修复..."

# 1. 修正 C++ 源文件中的模块名定义
# 使用 sed 命令将 PYBIND11_MODULE(aubo_py2, m) 替换为 aubo_py3
if grep -q "PYBIND11_MODULE(aubo_py2" aubo_wrapper.cpp; then
    sed -i 's/PYBIND11_MODULE(aubo_py2,/PYBIND11_MODULE(aubo_py3,/g' aubo_wrapper.cpp
    echo "[成功] 已修改 aubo_wrapper.cpp 中的模块名为 aubo_py3"
else
    echo "[跳过] aubo_wrapper.cpp 似乎已经修改过了"
fi

# 2. 清理旧的构建文件夹
echo ">>> 清理构建环境..."
rm -rf build
mkdir build
cd build

# 3. 编译
echo ">>> 开始编译..."
cmake ..
make

# 4. 检查结果并移动
if [ -f "aubo_py3.cpython"*".so" ] || [ -f "aubo_py3.so" ]; then
    echo ">>> 编译成功！正在安装..."
    cp aubo_py3*.so ../aubo_py3.so
    echo ">>> 全部完成！生成的库文件已更新为 aubo_py3.so"
else
    echo "[错误] 编译失败，未找到生成的 .so 文件。"
    exit 1
fi