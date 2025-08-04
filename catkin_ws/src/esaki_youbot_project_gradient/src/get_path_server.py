#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetPlan, GetPlanResponse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def handle_make_plan(req):
    # レスポンスとして返すPathオブジェクトを作成
    path = Path()
    path.header = Header(frame_id="map")  # 適切なヘッダーを設定

    # ダミーの経路をPathに追加
    # 実際の経路計画アルゴリズムに応じて経路を生成する
    start_pose = PoseStamped(header=Header(frame_id="map"), pose=req.start.pose)
    goal_pose = PoseStamped(header=Header(frame_id="map"), pose=req.goal.pose)
    path.poses.append(start_pose)
    path.poses.append(goal_pose)

    # 経路を含むレスポンスを返す
    return GetPlanResponse(plan=path)

def make_plan_server():
    rospy.init_node('make_plan_server')
    s = rospy.Service('make_plan', GetPlan, handle_make_plan)
    rospy.loginfo("Ready to make plan.")
    rospy.spin()

if __name__ == "__main__":
    make_plan_server()
