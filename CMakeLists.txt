cmake_minimum_required(VERSION 2.8.4)
project(odometry_transformer)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf2_geometry_msgs
  tf2_ros geometry_msgs
  dynamic_reconfigure
  nodelet
  pluginlib
  )

include_directories(include ${catkin_INCLUDE_DIRS})

#######################
# DYNAMIC RECONFIGURE #
#######################
generate_dynamic_reconfigure_options(cfg/OdometryTransformer.cfg)

catkin_package()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} src/odometry_transformer.cc
                            src/odometry_transformer_nodelet.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# Binaries #
############
add_executable(odometry_transformer_node src/odometry_transformer_node.cc)
target_link_libraries(odometry_transformer_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(odometry_transformer_node ${PROJECT_NAME}_gencfg)

add_executable(test_odometry_publisher test/test_odometry_publisher.cc)
target_link_libraries(test_odometry_publisher ${catkin_LIBRARIES})

##########
# EXPORT #
##########
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
