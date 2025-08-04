#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def move_base_client(x, y):
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
    goal.target_pose.pose.orientation.w = 1.0

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

        # 円周上の目標位置を設定する
        num_points = 10
        radius = 0.5
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = radius * math.cos(angle)
            y = -radius * math.sin(angle)
            result = move_base_client(x, y)
            if result:
                rospy.loginfo("Goal execution done!")

        # 最後に元の位置に戻る
        result = move_base_client(0.0, 0.0)
        if result:
            rospy.loginfo("Goal execution done!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")