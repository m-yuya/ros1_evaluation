cmake_minimum_required(VERSION 2.8.3)
project(nodelet_eval)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES nodelet_eval
  CATKIN_DEPENDS nodelet roscpp std_msgs
)

## Create the nodelet tutorial library
add_library(nodelet_eval src/talker_nodelet.cpp src/listener_nodelet.cpp)
target_link_libraries(nodelet_eval ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_eval ${catkin_EXPORTED_LIBRARIES})
endif()

## Create the nodelet tutorial library
# add_library(nodelet_listener src/listener.cpp)
# target_link_libraries(nodelet_listener ${catkin_LIBRARIES})
# if(catkin_EXPORTED_LIBRARIES)
#   add_dependencies(nodelet_listener ${catkin_EXPORTED_LIBRARIES})
# endif()


## Mark the nodelet library for installations
install(TARGETS nodelet_eval
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(TARGETS nodelet_listener
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
## install(FILES nodelet_math.xml talker.launch talker_default.yaml
##   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
