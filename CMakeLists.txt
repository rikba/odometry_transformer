cmake_minimum_required(VERSION 2.8.4)
project(odometry_transformer)

find_package(catkin REQUIRED
  COMPONENTS roscpp nav_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} src/odometry_transformer.cc)
target_link_libraries(${PROJECT_NAME})

############
# Binaries #
############
add_executable(odometry_transformer_node src/odometry_transformer_node.cc)
target_link_libraries(odometry_transformer_node ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
