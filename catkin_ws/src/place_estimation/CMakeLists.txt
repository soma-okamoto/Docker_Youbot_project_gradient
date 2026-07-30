cmake_minimum_required(VERSION 3.0.2)
project(place_estimation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/prior_distribution_node.py
    scripts/observation_fusion_node_multi_sensor.py
    scripts/distribution_visualization_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
