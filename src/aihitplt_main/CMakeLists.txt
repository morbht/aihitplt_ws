# 文件: aihitplt_main/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)
project(aihitplt_main)

## 编译选项
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  aihitplt_yolo
)


# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs aihitplt_yolo
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 添加可执行文件
add_executable(emergency_node
  src/emergency_node.cpp
)

## 链接库
target_link_libraries(emergency_node
  ${catkin_LIBRARIES}
)

## 添加依赖 
add_dependencies(emergency_node
  ${catkin_EXPORTED_TARGETS}
)

## 安装可执行文件
install(TARGETS emergency_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
