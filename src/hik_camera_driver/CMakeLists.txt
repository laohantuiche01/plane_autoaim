cmake_minimum_required(VERSION 3.8)
project(hik_camera_driver)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

# 1. 将核心代码和 Component 注册逻辑编译为一个动态库
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/hik_camera_driver_component_node.cpp
        src/hik_camera_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        /opt/MVS/include
        ${OpenCV_INCLUDE_DIRS}
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(STATUS "Your Device arch is x86 using /opt/MVS/lib/64 as camera library")
    target_link_directories(${PROJECT_NAME} PUBLIC /opt/MVS/lib/64)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "Your Device arch is aarch64 using /opt/MVS/lib/aarch64 as camera library")
    target_link_directories(${PROJECT_NAME} PUBLIC /opt/MVS/lib/aarch64)
else ()
    message(SEND_ERROR "Your Device arch is not supported!")
endif()

target_link_libraries(${PROJECT_NAME} MvCameraControl ${OpenCV_LIBS})
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

# 注册 Component (这也会根据 ament_auto 的特性处理 component 节点)
rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN HikCameraDriver
        EXECUTABLE ${PROJECT_NAME}_component_node
)

# 2. 编译普通 Node 节点
ament_auto_add_executable(${PROJECT_NAME}_node
        src/hik_camera_driver_node.cpp
)
# 直接将其链接到动态库上，它将继承所有的 PUBLIC includes 和 libraries
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
