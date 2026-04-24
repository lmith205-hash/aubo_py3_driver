cmake_minimum_required(VERSION 3.0)
project(aubo_py3_driver)

# 1. 寻找 pybind11 和 Python3
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# 2. 包含头文件
# 指向 SDK 头文件
include_directories(${PROJECT_SOURCE_DIR}/include/aubo_driver)
# 指向 Python3 头文件
include_directories(${Python3_INCLUDE_DIRS})

# 3. 链接库文件路径 (SDK 的 .so 所在位置)
link_directories(${PROJECT_SOURCE_DIR}/lib/lib64/aubocontroller)

# 4. 生成模块
# 注意：这里我们定义模块名为 aubo_py3
pybind11_add_module(aubo_py3 aubo_wrapper.cpp)

# 5. 链接具体的 SDK 库
target_link_libraries(aubo_py3 PRIVATE auborobotcontroller)

message(STATUS "Using Python version: ${Python3_VERSION}")