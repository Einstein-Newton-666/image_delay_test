cmake_minimum_required(VERSION 3.8)
project(image_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)

include_directories(${Python_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

ament_auto_add_library(image_sub SHARED src/image_sub.cpp)

rclcpp_components_register_node(image_sub PLUGIN image_test::image_sub EXECUTABLE image_sub_node)

ament_auto_add_library(image_pub SHARED src/image_pub.cpp)

rclcpp_components_register_node(image_pub PLUGIN image_test::image_pub EXECUTABLE image_pub_node)


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

ament_auto_package(INSTALL_TO_SHARE launch)
