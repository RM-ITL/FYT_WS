cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
DIRECTORY src
)

ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::serial_driver::SerialDriverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::serial_driver::VirtualSerialNode
  EXECUTABLE virtual_serial_node
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)

  ament_add_gtest(test_fixed_packet_tool test/test_fixed_packet_tool.cpp)
  target_link_libraries(test_fixed_packet_tool ${PROJECT_NAME})
endif()

#############
## Install ##
#############
set(USE_SCOPED_HEADER_INSTALL_DIR ON)

ament_auto_package(
  INSTALL_TO_SHARE
)
