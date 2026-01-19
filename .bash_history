cd catkin_ws
source devel/setup.bash
cd catkin_ws
source devel/setup.bash
rosrun esaki_youbot_project_gradient youbot_real_trajectory_node.py
cd catkin_ws/
source devel/setup.bash
roscore
cd catkin_ws
source devel/setup.bash
rosrun esaki_youbot_project_gradient IRM_youbot_baseMove.py
