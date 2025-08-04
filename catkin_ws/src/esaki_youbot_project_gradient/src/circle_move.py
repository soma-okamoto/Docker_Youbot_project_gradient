#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_current_pose():
    # 現在のロボットの位置を取得する
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    trans = tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
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

if __name__ == '__main__':
    try:
        # ノードを初期化する
        rospy.init_node('move_base_test', anonymous=True)

        # 初期位置から半径1mの円周上の目標位置を設定する
        num_points = 12
        radius = 0.25
        current_pose = get_current_pose()
        for i in range(num_points):

            angle = 2 * math.pi * i / num_points
            x = current_pose.pose.position.x + radius * math.cos(angle)
            y = current_pose.pose.position.y - radius * math.sin(angle)
            yaw = - angle - math.pi/2
            move_base_client(x, y, yaw)

            #版時計周り
            # angle = 2 * math.pi * i / num_points
            # x = current_pose.pose.position.x - radius * math.cos(angle)
            # y = current_pose.pose.position.y - radius * math.sin(angle)
            # yaw =  angle - math.pi/2
            # move_base_client(0, 0,0)

        # 最後に元の位置に戻る
        move_base_client(current_pose.pose.position.x, current_pose.pose.position.y, 0)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
