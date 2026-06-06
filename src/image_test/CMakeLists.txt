cmake_minimum_required(VERSION 3.8)
project(image_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 性能优化：启用 O3 和原生指令集（AVX2 等加速 memcpy）
add_compile_options(-O3 -march=native)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)
find_package(Boost REQUIRED system filesystem date_time iostreams serialization)
find_package(VTK REQUIRED)
find_package(libusb REQUIRED)

find_package(shm_msgs REQUIRED)
find_package(iceoryx_posh REQUIRED)

include_directories(
  ${Python_INCLUDE_DIRS}
  # 添加 shm_msgs 的头文件路径
  ${shm_msgs_INCLUDE_DIRS}
)


include_directories(${Python_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

ament_auto_add_library(image_sub SHARED src/image_sub.cpp)

target_include_directories(image_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(image_sub PLUGIN image_test::image_sub EXECUTABLE image_sub_node)
target_link_libraries(image_sub iceoryx_posh::iceoryx_posh)

ament_auto_add_library(image_pub SHARED src/image_pub.cpp)

target_link_libraries(image_pub ${shm_msgs_LIBRARIES} iceoryx_posh::iceoryx_posh)
target_include_directories(image_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(image_pub PLUGIN image_test::image_pub EXECUTABLE image_pub_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_image_pub_utils test/test_image_pub_utils.cpp)
  target_include_directories(test_image_pub_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_image_pub_utils rclcpp shm_msgs OpenCV)

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
