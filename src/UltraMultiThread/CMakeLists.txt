cmake_minimum_required(VERSION 3.8)
project(UltraMultiThread)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic -O3)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake_auto REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)
# find_package(pybind11 REQUIRED)

include_directories(${Python_INCLUDE_DIRS})
# include_directories(${pybind11_INCLUDE_DIRS})

# target_link_libraries(example pthread)
# target_link_libraries(example Python::Python pybind11::pybind11)

ament_auto_find_build_dependencies()

ament_auto_add_library(example_sub SHARED src/example_sub.cpp)
rclcpp_components_register_node(example_sub PLUGIN umt_example::example_sub EXECUTABLE example_sub_node)

ament_auto_add_library(example_pub SHARED src/example_pub.cpp)
rclcpp_components_register_node(example_pub PLUGIN umt_example::example_pub EXECUTABLE example_pub_node)

# ament_auto_add_executable(example_sub src/example_sub.cpp)

# ament_auto_add_executable(example_pub src/example_pub.cpp)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE include launch)