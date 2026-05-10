cmake_minimum_required(VERSION 3.8)
project(rm_camera_driver)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(camera_info_manager REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error=unused-variable)
  add_compile_options(-Wno-error=unused-parameter)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
##  src/daheng_camera.cpp
  src/hikcamera.cpp
  src/recorder.cpp
  src/video_player.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include 
  /opt/MVS/include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  camera_info_manager
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC lib/x86_64)
  install(
    DIRECTORY lib/x86_64
    DESTINATION lib
  )
#elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
#  install(
#    DIRECTORY hikSDK/lib/arm64/
#    DESTINATION lib
#  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

set(HIKSDK_LIB_PATH "/home/hou/opt/MVS/lib/64")
link_directories(${HIKSDK_LIB_PATH})

target_link_libraries(${PROJECT_NAME}
##  libgxiapi.so
  /opt/MVS/lib/64/libMvCameraControl.so
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  image_transport::image_transport
)

#rclcpp_components_register_node(${PROJECT_NAME}
#  PLUGIN fyt::camera_driver::DahengCameraNode
#  EXECUTABLE ${PROJECT_NAME}_node
#)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN camera::HikRobotNode
  EXECUTABLE hikvision_camera_node  # 节点可执行文件名称不变（或改为 hikvision_camera_node）
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::camera_driver::VideoPlayerNode
  EXECUTABLE video_player_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
