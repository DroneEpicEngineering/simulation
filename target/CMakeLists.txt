cmake_minimum_required(VERSION 3.8)
project(target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package(sim_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(TARGET_SYSTEM_SRC 
  src/target_system_node.cpp
  src/target_system_node.hpp
  src/trajectory_reader.cpp
  src/trajectory_reader.hpp
)

add_library(target_system_plugin SHARED ${TARGET_SYSTEM_SRC} src/target_system.cpp)
ament_target_dependencies(target_system_plugin
  rclcpp
  rclcpp_action
  sim_interfaces
  std_msgs
  geometry_msgs
)
target_link_libraries(target_system_plugin
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
install(TARGETS target_system_plugin LIBRARY DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
