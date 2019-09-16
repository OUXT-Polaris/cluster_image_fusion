cmake_minimum_required(VERSION 2.8.3)
project(cluster_image_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hungarian_solver
  message_filters
  nodelet
  pluginlib
  roscpp
  vision_msgs
  vision_info_server
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ClusterImageFusion.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cluster_image_fusion
  CATKIN_DEPENDS hungarian_solver message_filters nodelet pluginlib roscpp vision_msgs dynamic_reconfigure vision_info_server
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(cluster_image_fusion src/cluster_image_fusion.cpp)
target_link_libraries(cluster_image_fusion ${catkin_LIBRARIES})
class_loader_hide_library_symbols(cluster_image_fusion)

add_executable(cluster_image_fusion_node src/cluster_image_fusion_node.cpp)
target_link_libraries(cluster_image_fusion_node ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS cluster_image_fusion_node cluster_image_fusion
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)