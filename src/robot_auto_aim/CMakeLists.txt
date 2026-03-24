cmake_minimum_required(VERSION 3.10)
project(robot_auto_aim)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(G2O REQUIRED)
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)
find_package(jemalloc REQUIRED)

ament_auto_find_build_dependencies()

# Build
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/armor_detector_node.cpp
        src/armor_detector.cpp
        src/armor_pose_estimator.cpp
        src/ba_solver.cpp
        src/graph_optimizer.cpp
        src/light_corner_corrector.cpp
        src/number_classifier.cpp
        src/robot_target.cpp
        src/outpost_target.cpp
        src/tracker.cpp
        src/armor_solver_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o_core
        g2o_stuff
        g2o_solver_csparse
        g2o_types_sba
        g2o_types_slam3d
        g2o_solver_dense
        fmt::fmt
        tbb
        jemalloc
)

# 1. 注册Component，并分配一个不冲突的可执行文件名称
rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN robot_auto_aim::ArmorDetectorNode
        EXECUTABLE ${PROJECT_NAME}_detector_component_node
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN robot_auto_aim::ArmorSolverNode
        EXECUTABLE ${PROJECT_NAME}_solver_component_node
)

# 2. 编译带有main函数的独立可执行节点
ament_auto_add_executable(armor_detector_node
        src/armor_detector_main.cpp
)
target_link_libraries(armor_detector_node ${PROJECT_NAME})

ament_auto_add_executable(armor_solver_node
        src/armor_solver_main.cpp
)
target_link_libraries(armor_solver_node ${PROJECT_NAME})

ament_auto_add_executable(auto_aim_node
        src/auto_aim_main.cpp
)
target_link_libraries(auto_aim_node ${PROJECT_NAME})

# Testing
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
            ament_cmake_copyright
            ament_cmake_uncrustify
            ament_cmake_cpplint
            ament_cmake_lint_cmake
    )
    ament_lint_auto_find_test_dependencies()
endif ()

# Install
ament_auto_package(
        USE_SCOPED_HEADER_INSTALL_DIR
        INSTALL_TO_SHARE
        model
)
