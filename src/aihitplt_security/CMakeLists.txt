cmake_minimum_required(VERSION 3.0.2)
project(aihitplt_security)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

# 正确查找OpenCV
find_package(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 正确设置库文件路径
link_directories(
  ${catkin_LIB_DIRS}
  ${PROJECT_SOURCE_DIR}/lib
)

# 添加可执行文件
add_executable(Pan_Tilt_Camera_NODE 
  src/Alarm.cpp 
  src/ConfigParams.cpp  
  src/main.cpp  
  src/tool.cpp
  src/CapPicture.cpp  
  src/GetStream.cpp   
  src/playback.cpp  
  src/Voice.cpp
)

target_link_libraries(Pan_Tilt_Camera_NODE 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libhcnetsdk.so
  ${PROJECT_SOURCE_DIR}/lib/libHCCore.so
  ${PROJECT_SOURCE_DIR}/lib/libPlayCtrl.so
  ${PROJECT_SOURCE_DIR}/lib/libAudioRender.so
  ${PROJECT_SOURCE_DIR}/lib/libSuperRender.so
  pthread
  dl
  crypto
  ssl
)


# target_link_libraries(Pan_Tilt_Camera_NODE 
#   ${catkin_LIBRARIES} 
#   ${OpenCV_LIBRARIES}
#   hcnetsdk
#   HCCore
#   PlayCtrl
#   AudioRender
#   SuperRender
#   pthread
#   dl
#   crypto
#   ssl
# )

# 设置运行时库路径
set_target_properties(Pan_Tilt_Camera_NODE PROPERTIES
  INSTALL_RPATH "${PROJECT_SOURCE_DIR}/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
  SKIP_BUILD_RPATH FALSE
)

install(TARGETS Pan_Tilt_Camera_NODE
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY sdkLog
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY record
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY img
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
