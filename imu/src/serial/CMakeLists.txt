cmake_minimum_required(VERSION 3.8)
project(serial)

# 设置C++标准为C++17（ROS2推荐）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 启用严格的编译警告，帮助发现潜在问题
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)


set(SERIAL_SRCS
  src/serial.cc
)

# Unix/Linux/macOS 平台的源文件
if(UNIX)
  # Unix实现文件
  list(APPEND SERIAL_SRCS src/impl/unix.cc)

  if(APPLE)

    list(APPEND SERIAL_SRCS src/impl/list_ports/list_ports_osx.cc)
  else()
    # Linux系统
    list(APPEND SERIAL_SRCS src/impl/list_ports/list_ports_linux.cc)
  endif()
  
# Windows 平台的源文件
elseif(WIN32)
  list(APPEND SERIAL_SRCS src/impl/win.cc)
  list(APPEND SERIAL_SRCS src/impl/list_ports/list_ports_win.cc)
endif()


add_library(${PROJECT_NAME} SHARED ${SERIAL_SRCS})

# 设置库的头文件包含路径
# PUBLIC表示使用这个库的其他目标也能看到这些头文件
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(UNIX)
  # Unix/Linux需要pthread线程库（用于互斥锁）
  target_link_libraries(${PROJECT_NAME} pthread)
  
  if(APPLE)
    # macOS需要IOKit和CoreFoundation框架（用于枚举串口设备）
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
  elseif(NOT ANDROID)
    # Linux系统需要实时库rt（用于高精度时间函数）
    target_link_libraries(${PROJECT_NAME} rt)
  endif()
  
elseif(WIN32)
  # Windows需要setupapi库（用于枚举串口设备）
  target_link_libraries(${PROJECT_NAME} setupapi)
endif()

if(APPLE)
  # macOS需要定义这些宏以使用正确的API
  target_compile_definitions(${PROJECT_NAME} 
    PRIVATE
      __APPLE__
      MAC_OS_X_VERSION_MIN_REQUIRED=MAC_OS_X_VERSION_10_9
  )
endif()


install(
  DIRECTORY include/
  DESTINATION include
)

# 安装编译好的库文件
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib          # Linux/macOS的.so文件
  ARCHIVE DESTINATION lib          # Windows的.lib文件
  RUNTIME DESTINATION bin          # Windows的.dll文件
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 导出包含路径
ament_export_include_directories(include)

# 导出库依赖（让使用者知道需要链接哪些系统库）
if(UNIX AND NOT APPLE)
  ament_export_libraries(pthread rt)
elseif(APPLE)
  ament_export_libraries(pthread)
elseif(WIN32)
  ament_export_libraries(setupapi)
endif()

ament_package()