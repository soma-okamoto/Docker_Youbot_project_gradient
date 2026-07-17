######################################################################################################
#################################################################################################
#ホスト側

PC２

export ROS_MASTER_URI=http://192.168.11.14:11311
export ROS_IP=192.168.11.14
export ROS_HOSTNAME=192.168.11.14

roslaunch rosbridge_server rosbridge_websocket.launch

# PC３ の IP
export ROS_IP=192.168.11.6
export ROS_MASTER_URI=http://192.168.11.14:11311



####################################################################################

PC3手順
#####################################################################
###############################Youbotsim#######################
cd Doceker_ws/Docker_youbot_sim
export ROS_IP=192.168.11.13
export ROS_MASTER_URI=http://192.168.11.30:11311
source devel/setup.bash

roslaunch youbot_gazebo_robot youbot_dual_arm.launch world:=empty_world

############################################################################################
# 1) Docker_ReachabilityMapターミナル#########################################################
export ROS_IP=192.168.11.10
export ROS_MASTER_URI=http://192.168.11.14:11311


docker rm -f irm_dev
docker run -d --name irm_dev --network=host \
  -v ~/Doceker_ws/Docker_ReachabilityMap:/root \
  -e ROS_MASTER_URI=http://192.168.11.17:11311/ \
  -e ROS_IP=192.168.11.16  \
  irm_dev tail -f /dev/null


docker exec -it irm_dev bash
cd RM
export ROS_IP=192.168.11.22
export ROS_MASTER_URI=http://192.168.11.2:11311
source devel/setup.bash
roslaunch sampled_reachability_maps MR_IRM_generate_Docker.launch


docker exec -it irm_dev bash
cd Detect_ws
export ROS_IP=192.168.11.16
export ROS_MASTER_URI=http://192.168.11.17:11311
source devel/setup.bash
rosrun detect_pkg DetectTarget.py \
  --win=0.5,0.25,0.25 \
  --wout=0.21,0.58,0.21


############################################################################################
# 1) youbootターミナル##############################################################################
export ROS_IP=192.168.11.16
export ROS_MASTER_URI=http://192.168.11.17:11311


docker rm -f youbot_pro
docker run -d --name youbot_pro --network=host \
  -v ~/Doceker_ws/Docker_Youbot_project_gradient:/root \
  -e ROS_MASTER_URI=http://192.168.11.48:11311 \
  -e ROS_IP=192.168.11.47 \
  youbot_pro tail -f /dev/null


docker exec -it youbot_pro bash
cd catkin_ws
export ROS_IP=192.168.11.47
export ROS_MASTER_URI=http://192.168.11.48:11311
source devel/setup.bash



#####mani
rosrun esaki_youbot_project_gradient youbot_real_trajectory_node.py
rosrun esaki_youbot_project_gradient youbot_real_trajectory_node_FMS.py
# rosrun esaki_youbot_project_gradient gripper.py 

rosrun esaki_youbot_project_gradient youbot_camera_real_trajectory_node.py 

右アーム追従＆tf
rosrun esaki_youbot_project_gradient youbot_camera_trajectory_TF.py 


###SimBridge
rosrun esaki_youbot_project_gradient Bridge_Simulation_command.py


# ####Slam
# roslaunch esaki_slam youbot_move_base.launch
# roslaunch slam_toolbox online_async.launch 


# ####ベース移動
# rosrun esaki_youbot_project_gradient IRM_youbot_baseMove.py
# rosrun esaki_youbot_project_gradient Origin_move_pub.py
# rosrun esaki_youbot_project_gradient aster_static.py


# ######Log
# rosrun esaki_youbot_project_gradient ybt_metrics_csv_logger.py 
 
# docker cp youbot_pro:/tmp/ybt_metrics_20250828_204509.csv .


####PointCloudの取得・合成(youbotが動く)

rosrun esaki_youbot_project_gradient move_base_global_registration.py


#####キャリブレーション(起動状態でMR操作)
rosrun esaki_youbot_project_gradient afine_transformation.py


########
catkin clean -f 
 


############################################################################################
########################################################################################################

cd realsense_ws
export ROS_IP=192.168.11.47
export ROS_MASTER_URI=http://192.168.11.48:11311
source devel/setup.bash

roslaunch realsense2_camera cubeslam_camera.launch 

####################RTX5070 Yolov5_deep_sort
docker rm -f yolov5-strongsort:rtx5070-cu128

sudo -E env \
  ROS_MASTER_URI=http://192.168.11.48:11311 \
  ROS_IP=192.168.11.47 \
  ./run_rtx5070.sh

source /opt/ros/noetic/setup.bash
source /opt/ros_py310/setup.bash
source /home/dars/catkin_ws/devel/setup.bash

cd /home/dars/catkin_ws/src/Yolov5_StrongSORT/Yolov5_StrongSORT_OSNet
rosrun Yolov5_StrongSORT track_save_gpu.py \
  --device 0 \
  --view-img

＃これarm2のcommandないとエラーはく
 

rosrun Yolov5_StrongSORT QRPostion_test.py 







#####################################################################
############################################################
find . -name "*.py" -exec chmod +x {} \;

sudo ip route add 10.42.0.0/24 via 192.168.11.47


sudo ntpdate -u ccntp.meijo-u.ac.jp





##############AMIR##################################################################################
xhost +local:

docker rm -f humble_dev
docker run -d --name humble_dev --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=41 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/ros2_humble_ws/src/humbleble_ws:/home/dev/ws \
  humble_dev tail -f /dev/null


docker exec -it humble_dev bash

cd ~/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

gazebo 立ち上げ
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amir_gazebo gazebo_bringup.launch.py


Servo 一式起動 (servo_node + joint_state_filter + forward_position_controller を --inactive で spawn)
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amir_operation vr_servo_launch.py


出力先を Servo へ切替 (JTC を止めて forward_position_controller を有効化)
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller

# 1. Servo停止
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger {}

# 2. 初期姿勢に戻す
ros2 topic pub -r 10 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.428950932982121, 0.5292646205734075, -1.132436603496916, -0.7451734776362945, 0.0]}"

# 数秒後 Ctrl+C

# 3. Servo状態リセット
ros2 service call /servo_node/reset_servo_status std_srvs/srv/Trigger {}
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}


キーボード操作 (3DOF位置ジョグ版 / 5軸で並進だけ素直に動かす)【推奨】
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amir_operation servo_keyboard_jog 


自律動作 (JTC) へ戻す
source install/setup.bash
ros2 control switch_controllers --deactivate forward_position_controller --activate arm_controller





rosrun esaki_youbot_project_gradient AMIR_Keybord.py \
  _joint_names:="['Joint_1','Joint_2','Joint_3','Joint_4','Joint_5']" \
  _speed:=0.20 \
  _rate:=60.0 \
  _duration:=0.05

rosrun esaki_youbot_project_gradient AMIR_real_trajectory.py 

ros2 run amir_operation amir_gripper.py

ros2 run amir_operation amir_real_trajectory.py
