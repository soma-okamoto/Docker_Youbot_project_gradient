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
export ROS_IP=192.168.11.11
export ROS_MASTER_URI=http://192.168.11.14:11311
source devel/setup.bash

roslaunch youbot_gazebo_robot youbot_dual_arm.launch world:=empty_world

############################################################################################
# 1) Docker_ReachabilityMapターミナル#########################################################
export ROS_IP=192.168.11.10
export ROS_MASTER_URI=http://192.168.11.14:11311


docker rm -f irm_dev
docker run -d --name irm_dev --network=host \
  -v ~/Doceker_ws/Docker_ReachabilityMap:/root \
  -e ROS_MASTER_URI=http://192.168.11.14:11311/ \
  -e ROS_IP=192.168.11.11  \
  irm_dev tail -f /dev/null


docker exec -it irm_dev bash
cd RM
export ROS_IP=192.168.11.11
export ROS_MASTER_URI=http://192.168.11.14:11311
source devel/setup.bash
roslaunch sampled_reachability_maps MR_IRM_generate_Docker.launch


docker exec -it irm_dev bash
cd Detect_ws
export ROS_IP=192.168.11.11
export ROS_MASTER_URI=http://192.168.11.14:11311
source devel/setup.bash
rosrun detect_pkg DetectTarget.py \
  --win=0.5,0.25,0.25 \
  --wout=0.21,0.58,0.21


############################################################################################
# 1) youbootターミナル##############################################################################
export ROS_IP=192.168.11.6
export ROS_MASTER_URI=http://192.168.11.14:11311


docker rm -f youbot_pro
docker run -d --name youbot_pro --network=host \
  -v ~/Doceker_ws/Docker_Youbot_project_gradient:/root \
  -e ROS_MASTER_URI=http://192.168.11.14:11311 \
  -e ROS_IP=192.168.11.11 \
  youbot_pro tail -f /dev/null


docker exec -it youbot_pro bash
cd catkin_ws
export ROS_IP=192.168.11.11
export ROS_MASTER_URI=http://192.168.11.14:11311
source devel/setup.bash



#####mani
rosrun esaki_youbot_project_gradient youbot_real_trajectory_node.py
rosrun esaki_youbot_project_gradient youbot_real_trajectory_node_FMS.py
# rosrun esaki_youbot_project_gradient gripper.py 

rosrun esaki_youbot_project_gradient youbot_camera_real_trajectory_node.py 


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
source /opt/ros/noetic/setup.bash
catkin build



############################################################################################
########################################################################################################
######yolov5

cd catkin_ws
export ROS_IP=192.168.11.11
export ROS_MASTER_URI=http://192.168.11.14:11311
source devel/setup.bash

cd src/Yolov5_StrongSORT/Yolov5_StrongSORT_OSNet/


# rosrun Yolov5_StrongSORT track_1.py
# rosrun Yolov5_StrongSORT track_1.py --device cpu

# rosrun Yolov5_StrongSORT track.py --device cpu

＃これarm2のcommandないとエラーはく
 
rosrun Yolov5_StrongSORT track_save_cpu.py 



# conda activate yolo_env
# export ROS_MASTER_URI=http://192.168.11.14:11311
# export ROS_IP=192.168.11.6
# CUDA_FORCE_PTX_JIT=1 TORCH_CUDA_ARCH_LIST="9.0" python track_1_cal.py


#####################################################################
############################################################
find . -name "*.py" -exec chmod +x {} \;

sudo ip route add 10.42.0.0/24 via 192.168.11.14
