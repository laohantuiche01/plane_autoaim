cmake_minimum_required(VERSION 3.8)
project(robot_communication)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_subdirectory(thirdparty/serialib)
include_directories(include thirdparty/serialib)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(robot_communication_node src/robot_communication_node.cpp)
target_link_libraries(robot_communication_node serialib)
target_compile_features(robot_communication_node PUBLIC c_std_99 cxx_std_20)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

