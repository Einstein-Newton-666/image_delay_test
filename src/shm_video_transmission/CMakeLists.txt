cmake_minimum_required(VERSION 3.8)
project(shm_video_transmission)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic -O3)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

add_executable(sender src/demo_sender.cpp)
target_link_libraries(sender ${OpenCV_LIBS} ${Boost_LIBRARIES} pthread rt)

add_executable(receiver src/demo_receiver.cpp)
target_link_libraries(receiver ${OpenCV_LIBS} ${Boost_LIBRARIES} pthread rt)



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

ament_auto_package(INSTALL_TO_SHARE include)