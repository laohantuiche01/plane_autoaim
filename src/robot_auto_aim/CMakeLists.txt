cmake_minimum_required(VERSION 3.8)
project(robot_auto_aim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找 package.xml 中定义的依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 自动将 src 下的文件编译为库并作为组件注册
# 假设之后会创建 AutoAimNode.cpp
# ament_auto_add_library(${PROJECT_NAME} SHARED src/auto_aim_node.cpp)

# 如果有独立运行的节点
# ament_auto_add_executable(${PROJECT_NAME}_node src/main.cpp)

ament_auto_package()
