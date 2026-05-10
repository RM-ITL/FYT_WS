cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
add_definitions(-O3)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXTERNAL_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${G2O_LIBRARIES})

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
# 注释掉原有的 g2o 自动查找
# find_package(G2O REQUIRED)
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)
ament_auto_find_build_dependencies()

# 手动配置 g2o 路径
set(G2O_INCLUDE_DIRS /usr/local/include/g2o)
set(G2O_LIBRARY_DIRS /usr/local/lib)
link_directories(${G2O_LIBRARY_DIRS})
set(G2O_LIBRARIES
  g2o_core
  g2o_stuff
  g2o_solver_csparse
  g2o_types_sba
  g2o_types_slam3d
  g2o_solver_dense
)
# 验证路径（可选）
if(NOT EXISTS ${G2O_INCLUDE_DIRS})
  message(FATAL_ERROR "g2o include dir not found: ${G2O_INCLUDE_DIRS}")
endif()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  fmt  # 声明 fmt 为依赖
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  g2o_core
  g2o_stuff
  g2o_solver_csparse
  g2o_types_sba
  g2o_types_slam3d
  g2o_solver_dense
  fmt::fmt
  tbb
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
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
    ament_cmake_lint_cmake
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector ${PROJECT_NAME})

endif()

#############
## Install ##
#############
set(USE_SCOPED_HEADER_INSTALL_DIR ON)

ament_auto_package(
  INSTALL_TO_SHARE
  docs
  model
)
