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

def move_base_client(x, y, rot):
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

    goal.target_pose.pose.orientation.x = rot.x
    goal.target_pose.pose.orientation.y = rot.y
    goal.target_pose.pose.orientation.z = rot.z
    goal.target_pose.pose.orientation.w = rot.w

    # MoveBaseActionを実行する
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        
        return client.get_result()

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
    gool_pub = rospy.Publisher('gool_state',Bool,queue_size=1)
    
    youbot_pos_pub = rospy.Publisher('YouBot_Position', PoseStamped, queue_size=1)

    edit_pose_sub = rospy.Subscriber("edit_path_Command",Path,edit_path_callback)

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
        if(len(edit_path.poses) != 0):

            first_path = edit_path.poses

            for i in range(len(first_path)):
            #for pose in first_path:
                x = first_path[i].pose.position.x
                y = first_path[i].pose.position.y

                rot = first_path[i].pose.orientation
                result = move_base_client(x,y,rot)

                if result:
                    current_pose = get_current_pose()
                    youbot_pos_pub.publish(current_pose)
                    rospy.sleep(1)
            
            gool_status = True
            gool_pub.publish(gool_status)
            gool_status = False
            current_pose = get_current_pose()
            youbot_pos_pub.publish(current_pose)
            break
        rate.sleep()