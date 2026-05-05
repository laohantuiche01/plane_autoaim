cmake_minimum_required(VERSION 3.8)
project(robot_ballistics)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(ballistics_calculator src/ballistics_calculator.cpp)
target_include_directories(ballistics_calculator PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(ballistics_calculator Eigen3::Eigen)

add_library(ballistics_node SHARED
        src/ballistics_node.cpp
)
target_include_directories(ballistics_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(ballistics_node
        ballistics_calculator
)
ament_target_dependencies(ballistics_node
        rclcpp
        rclcpp_components
        Eigen3
        tf2_ros
        tf2_geometry_msgs
        robot_interfaces
        robot_utils
        visualization_msgs
        rclcpp_lifecycle
        sensor_msgs
        cv_bridge
        OpenCV
)

rclcpp_components_register_node(ballistics_node
        PLUGIN robot_ballistics::BallisticsNode
        EXECUTABLE ballistics_component_node
)

ament_export_targets(ballistics_calculator HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components Eigen3 tf2_ros tf2_geometry_msgs robot_interfaces robot_utils visualization_msgs sensor_msgs cv_bridge OpenCV)

install(TARGETS ballistics_calculator ballistics_node
        EXPORT ballistics_calculator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
