cmake_minimum_required(VERSION 3.8)
project(robot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/math_utils.cpp
  src/pnp_solver.cpp
  src/url_resolver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)

ament_export_include_directories(${EIGEN3_INCLUDE_DIRS})
ament_export_dependencies(OpenCV Eigen3)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
