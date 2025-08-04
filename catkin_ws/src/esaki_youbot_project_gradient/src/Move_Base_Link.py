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

class NearestSafePositionFinder:
    def __init__(self, safety_radius):
        self.safety_radius = safety_radius
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map_data = None

    def map_callback(self, msg):
        self.map_data = msg

    def is_safe_position(self, point):
        if self.map_data is None:
            return False

        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        # チェックするグリッドの範囲を計算
        min_x = int((point.x - self.safety_radius - origin.position.x) / resolution)
        max_x = int((point.x + self.safety_radius - origin.position.x) / resolution)
        min_y = int((point.y - self.safety_radius - origin.position.y) / resolution)
        max_y = int((point.y + self.safety_radius - origin.position.y) / resolution)

        # 指定された範囲内で障害物があるかチェック
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                if 0 <= x < self.map_data.info.width and 0 <= y < self.map_data.info.height:
                    index = y * self.map_data.info.width + x
                    if self.map_data.data[index] == 100:  # 障害物あり
                        print("Target Position not Safe")
                        return False
        
        print("Target Position Safe")
        return True
    
    def get_nearest_safe_position(self, target,origin,moveDistance):

        if self.is_safe_position(target):
            return target  # 基準位置がすでに安全であれば、それを返す
        directions = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]
        nearest_safe_position = None
        min_distance = float('inf')

        for dx, dy in directions:
            target_point = Point(
                target.x + dx * moveDistance,  # 0.1メートル進む
                target.y + dy * moveDistance,
                0
            )
            if self.is_safe_position(target_point):
                distance = math.sqrt((target_point.x - origin.x)**2 + (target_point.y - origin.y)**2)
                if distance < min_distance:
                    min_distance = distance
                    nearest_safe_position = target_point

        return nearest_safe_position
        
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
    q = quaternion_from_euler(0, 0, yaw)
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

def move_callback(msg):
    global pose
    #rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    pose = msg

if __name__ == '__main__':

    theta = 0

    accumulated_pc = []
    # ノードを初期化する
    rospy.init_node('move_base_link', anonymous=True)

    target = None
    source = None
    # cmd_vel トピックへのパブリッシャを作成
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    pose_sub = rospy.Subscriber("Base_Move",PoseStamped,move_callback)

    pose = PoseStamped()

    rate = rospy.Rate(10)

    gool_status = False
    result = False
    velocity_msg = Twist()
    safety_radius=0.2
    move_distance = 0.1
    while not rospy.is_shutdown():
        youbot_pos_pub = rospy.Publisher('YouBot_Position', PoseStamped, queue_size=1)
        gool_pub = rospy.Publisher('gool_state',Bool,queue_size=1)
        
        x = pose.pose.position.x
        y = pose.pose.position.y
        
        target_point = Point(x,y,0)
        if(x != 0 or y != 0):

            if(gool_status == False):
                current_pose = get_current_pose()
                #print("result : " + str(result))

                if(result == False):
                    
                    finder = NearestSafePositionFinder(safety_radius)  # 安全半径

                    rospy.sleep(1)  # 地図データの受信を待つ
                    point_Before = Point(current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z)
                    nearest_safe_position = finder.get_nearest_safe_position(target_point,point_Before,move_distance)
                    if nearest_safe_position:
                        print("Nearest safe position found at:", nearest_safe_position.x, nearest_safe_position.y)

                        result = move_base_client(nearest_safe_position.x,nearest_safe_position.y, theta)
                        print("result : " + str(result))
                        
                        if result:
                            rospy.sleep(1)
                            rospy.loginfo("Near Position  Done")
                    else:
                        move_distance +=0.1
                        print("No safe position found.")

                elif(result):
                    # 目的地までのXおよびYの差を計算
                    delta_x = x - current_pose.pose.position.x
                    delta_y = y - current_pose.pose.position.y
                    rospy.loginfo("target position Move")
                    if abs(delta_x) < 0.1 and abs(delta_y) < 0.1:  # 目的地に十分近づいたら停止
                        youbot_pos_pub.publish(current_pose)

                        gool_status = True
                        gool_pub.publish(gool_status)
                        gool_status = False

                        # 停止メッセージを送信
                        velocity_msg.linear.x = 0
                        velocity_msg.linear.y = 0
                        velocity_publisher.publish(velocity_msg)
                        rospy.loginfo("Target Position Goal execution Done")

                        break
                    
                    else:
                        # XおよびY方向の速度を設定
                        velocity_msg.linear.x = 0.5 * delta_x
                        velocity_msg.linear.y = 0.5 * delta_y
                        velocity_msg.angular.z = 0  # 回転しない

                        # 速度をパブリッシュ
                        velocity_publisher.publish(velocity_msg)

            rate.sleep()

    # rospy.init_node('nearest_safe_position_finder')
    # finder = NearestSafePositionFinder(safety_radius=0.5)  # 安全半径
    # # test_point = Point(-0.1, 0.60, 0)  # 基準となる位置
    # rospy.sleep(1)  # 地図データの受信を待つ
    # nearest_safe_position = finder.get_nearest_safe_position(test_point,0.1)
    # if nearest_safe_position:
    #     print("Nearest safe position found at:", nearest_safe_position.x, nearest_safe_position.y)
    # else:
    #     print("No safe position found.")
    

# if __name__ == '__main__':
#     rospy.init_node('safe_position_checker')
#     checker = SafePositionChecker(safety_radius=0.5)  # 安全半径を0.5メートルとする
#     test_point = Point(-0.1, 0.50, 0)  # テストする座標
#     rospy.sleep(1)  # 地図データの受信を待つ
#     if checker.is_safe_position(test_point):
#         print("The position is safe.")
#     else:
#         print("The position is not safe.")


# if __name__ == '__main__':
#     x = 0
#     y = 0
#     # theta = math.pi/6
#     theta = 0

#     accumulated_pc = []

#     # ノードを初期化する
#     rospy.init_node('move_base_link', anonymous=True)

#     target = None
#     source = None

#     pose_sub = rospy.Subscriber("Base_Move",PoseStamped,move_callback)

#     pose = PoseStamped()

#     rate = rospy.Rate(10)

#     gool_status = False
#     while not rospy.is_shutdown():
#         youbot_pos_pub = rospy.Publisher('YouBot_Position', PoseStamped, queue_size=1)
#         gool_pub = rospy.Publisher('gool_state',Bool,queue_size=1)
        
#         x = pose.pose.position.x
#         y = pose.pose.position.y
        
#         if(x != 0 or y != 0):
#             if(gool_status == False):

#                 result = move_base_client(x,y, theta)
                
#                 if result:
#                     current_pose = get_current_pose()
#                     youbot_pos_pub.publish(current_pose)

#                     gool_status = True
#                     gool_pub.publish(gool_status)

#                     gool_status = False

#                     rospy.loginfo("Target Position Goal execution Done")

#             rate.sleep()