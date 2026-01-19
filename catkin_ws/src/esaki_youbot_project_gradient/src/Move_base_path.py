#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import open3d as o3d
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header,String,Bool
from sensor_msgs import point_cloud2
import numpy as np
from tf import TransformListener
from geometry_msgs.msg import PointStamped,PoseStamped, Quaternion
import signal
import sys
import tf.transformations as tf_trans

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Path


        
def get_current_pose():
    # 現在のロボットの位置を取得する
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = trans.transform.translation.x
    pose.pose.position.y = trans.transform.translation.y
    pose.pose.position.z = trans.transform.translation.z
    pose.pose.orientation.x = trans.transform.rotation.x
    pose.pose.orientation.y = trans.transform.rotation.y
    pose.pose.orientation.z = trans.transform.rotation.z
    pose.pose.orientation.w = trans.transform.rotation.w
    return pose

def move_base_client(x, y, yaw):
    # MoveBaseActionクライアントを作成する
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # 目標位置を設定する
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 目標位置のx, y座標を設定する
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # YouBotの向きを設定する
    q = quaternion_from_euler(0, 0, 0)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    # MoveBaseActionを実行する
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        
        return client.get_result()

def path_callback(msg):
    global path
    #rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    path = msg

def edit_path_callback(msg):
    global edit_path
    #rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    edit_path = msg

if __name__ == '__main__':

    theta = 0

    accumulated_pc = []
    # ノードを初期化する
    rospy.init_node('move_base_link', anonymous=True)

    target = None
    source = None
    # cmd_vel トピックへのパブリッシャを作成
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    youbot_pos_pub = rospy.Publisher('YouBot_Position', PoseStamped, queue_size=1)
    gool_pub = rospy.Publisher('gool_state',Bool,queue_size=1)

    pose_sub = rospy.Subscriber("Start_Plan",Path,path_callback)
    
    edit_pose_sub = rospy.Subscriber("edit_path_Command",Path,edit_path_callback)

    path = Path()

    edit_path = Path()

    rate = rospy.Rate(10)

    gool_status = False
    result = False
    velocity_msg = Twist()
    safety_radius=0.2
    move_distance = 0.1

    while not rospy.is_shutdown():
        
        rospy.sleep(5)
        print("Move Start")
        if(len(path.poses) != 0):

            first_path = path.poses

            for i in range(len(first_path)):
            #for pose in first_path:
                
                while True:
                    x = first_path[i].pose.position.x
                    y = first_path[i].pose.position.y

                    # x = pose.pose.position.x
                    # y = pose.pose.position.y
                    current_pose = get_current_pose()
                    # 目的地までのXおよびYの差を計算
                    delta_x = x - current_pose.pose.position.x
                    delta_y = y - current_pose.pose.position.y
                    print("x : + " + str(x) + " : y : " + str(y))
                    
                    if abs(delta_x) < 0.1 and abs(delta_y) < 0.02:  # 目的地に十分近づいたら停止
                        # 停止メッセージを送信
                        velocity_msg.linear.x = 0
                        velocity_msg.linear.y = 0
                        velocity_publisher.publish(velocity_msg)
                        rospy.loginfo("Target Position Goal execution Done")
                        
                        if(len(edit_path.poses) != 0):
                         
                            first_path = edit_path.poses
                        break

                    else:
                        # XおよびY方向の速度を設定
                        velocity_msg.linear.x = 0.5 * delta_x
                        velocity_msg.linear.y = 0.5 * delta_y
                        velocity_msg.angular.z = 0  # 回転しない
                        print("velocity x : " + str(velocity_msg.linear.x) + "velocity y : " + str(velocity_msg.linear.y) )
                        # 速度をパブリッシュ
                        velocity_publisher.publish(velocity_msg)
                        rospy.loginfo("target position Move")
            
            gool_status = True
            gool_pub.publish(gool_status)
            gool_status = False
            current_pose = get_current_pose()
            print(current_pose)
            youbot_pos_pub.publish(current_pose)
            # if pose_count == path.poses.count -1:
            #     
            break
        rate.sleep()