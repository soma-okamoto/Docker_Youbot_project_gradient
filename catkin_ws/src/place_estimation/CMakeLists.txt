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
    scripts/observation_fusion_node_new.py
    scripts/distribution_visualization_node.py
    evaluation/kl_evaluation_node.py
    analysis/preliminary_experiment_analyzer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    evaluation/gaussian_kl.py
    evaluation/kl_trend.py
    analysis/preliminary_statistics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config docs analysis
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
