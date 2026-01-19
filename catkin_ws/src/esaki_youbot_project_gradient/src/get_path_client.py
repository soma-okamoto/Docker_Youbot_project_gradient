#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,Point
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan,GetPlanResponse,GetPlanRequest
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans
from tf import TransformListener
from nav_msgs.msg import OccupancyGrid
import math
from scipy.spatial import ConvexHull
from std_msgs.msg import Float32MultiArray

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
        
        target_point = Point(
                target[0],  # 0.1メートル進む
                target[1],
                0
            )

        if self.is_safe_position(target_point):
            return target_point  # 基準位置がすでに安全であれば、それを返す

        directions = [(0, 1)]
        nearest_safe_position = target_point
        min_distance = 0

        for i in range(50):

            for dx, dy in directions:
                target_point = Point(
                    target[0] + dx * moveDistance * (i + 1),  # 0.1メートル進む
                    target[1] + dy * moveDistance * (i + 1),
                    0
                )
                if self.is_safe_position(target_point):
                    distance = math.sqrt((target_point.x - origin.x)**2 + (target_point.y - origin.y)**2)
                    if distance > min_distance:
                        min_distance = distance
                        nearest_safe_position = target_point
        return nearest_safe_position

def optimalPosition(current_pose,pick_pos):
    # 初期位置
    initial_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y])

    Pick_pos = []
   
    for i in range(int(len(pick_pos.data)/3)):
        pos = []
        pos.append(- pick_pos.data[i * 3])
        pos.append(- pick_pos.data[i*3 +2])
        Pick_pos.append(pos)
        

    #TODO Pickしたいオブジェクトの座標を入れる。
    object_positions = np.array(Pick_pos)

    # print(object_positions)

    # ロボットのアームの最大到達距離
    max_reach = 0.5  # メートル

    # 安全距離
    safety_distance = 0.3  # メートル

    # 凸包の計算
    hull = ConvexHull(object_positions)
    hull_vertices = object_positions[hull.vertices]

    # 凸包の重心を計算
    hull_centroid = np.mean(hull_vertices, axis=0)

    # 最大到達距離と安全距離を考慮して最適な位置を探索
    optimal_position = hull_centroid  # 初期値として重心を設定
    min_distance_to_initial = np.inf

    # 位置の微調整
    for angle in np.linspace(0, 2 * np.pi, 100):
        for r in np.linspace(safety_distance, max_reach - safety_distance, 50):
            candidate_position = hull_centroid + r * np.array([np.cos(angle), np.sin(angle)])
            distances = np.linalg.norm(object_positions - candidate_position, axis=1)
            if np.all((distances <= max_reach) & (distances >= safety_distance)):
                distance_to_initial = np.linalg.norm(candidate_position - initial_position)
                if distance_to_initial < min_distance_to_initial:
                    min_distance_to_initial = distance_to_initial
                    optimal_position = candidate_position
                    print(optimal_position)

    # 各オブジェクトまでの距離を計算
    distances = np.linalg.norm(object_positions - optimal_position, axis=1)

    return optimal_position

def get_plan(x_start,y_start,x_goal,y_goal, yaw_goal):
    rospy.wait_for_service('/move_base/make_plan')
    try:
        # 経路計画サービスのクライアントを作成
        get_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        # # 開始位置を設定
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.position.x = x_start
        start.pose.position.y = y_start
        start.pose.orientation.w = 1.0  # 北を向く

        # # 目標位置を設定
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = x_goal
        goal.pose.position.y = y_goal

        q = quaternion_from_euler(0, 0, yaw_goal)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]


        # GetPlanRequest オブジェクトを作成
        get_plan_req = GetPlanRequest()
        get_plan_req.start = start
        get_plan_req.goal = goal
        get_plan_req.tolerance = 0.0  # 許容誤差を設定

        # サービスを呼び出し
        response = get_plan_service(get_plan_req)

        # 経路を返す
        return response.plan
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


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

def move_callback(msg):
    global goal
    # rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    goal = msg

def pick_object_callback(pick_msg):
    global pick_object_pose
    # rospy.loginfo("Message '{}' recieved".format(pick_msg))
    pick_object_pose = pick_msg


def calculate_distance(pose1, pose2):
    """2つのPoseStampedオブジェクト間の距離を計算する"""
    dx = pose1.pose.position.x - pose2.pose.position.x
    dy = pose1.pose.position.y - pose2.pose.position.y
    return math.sqrt(dx**2 + dy**2)

def is_turning_point(pose1, pose2, pose3, threshold_angle=10):
    """3つの連続するPoseStampedオブジェクトが曲がり角かどうかを判定する"""
    vector1 = (pose2.pose.position.x - pose1.pose.position.x, pose2.pose.position.y - pose1.pose.position.y)
    vector2 = (pose3.pose.position.x - pose2.pose.position.x, pose3.pose.position.y - pose2.pose.position.y)
    dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
    magnitude1 = math.sqrt(vector1[0]**2 + vector1[1]**2)
    magnitude2 = math.sqrt(vector2[0]**2 + vector2[1]**2)
    if magnitude1 == 0 or magnitude2 == 0:
        return False
    angle = math.acos(dot_product / (magnitude1 * magnitude2))
    angle_degrees = math.degrees(angle)
    return abs(angle_degrees) > threshold_angle

def process_path(path):
    """経路を処理して1m間隔と曲がり角でウェイポイントをPublishする"""
    waypoints = []
    accumulated_distance = 0.0
    for i in range(1, len(path.poses),5):
        if i == 1:
            waypoints.append(path.poses[i - 1])
            continue

        distance = calculate_distance(path.poses[i-1], path.poses[i])
        accumulated_distance += distance
        if accumulated_distance >= 0.1:
            waypoints.append(path.poses[i])
            accumulated_distance = 0.0
            continue


        if i > 1 and is_turning_point(path.poses[i-2], path.poses[i-1], path.poses[i]):
            waypoints.append(path.poses[i-1])
            continue
    
    if(len(waypoints) > 1):
        distance = calculate_distance(path.poses[len(path.poses) - 2], path.poses[len(path.poses) -1])
    
        if(distance <= 0.1):
            waypoints.pop(len(waypoints) - 1)
    waypoints.append(path.poses[len(path.poses) - 1])
    return waypoints


if __name__ == "__main__":
    rospy.init_node('get_plan_client')
    rate = rospy.Rate(10)

    pose_sub = rospy.Subscriber("Base_Move",PoseStamped,move_callback)
    plan_pub = rospy.Publisher('Start_Plan', Path, queue_size=1)
    pick_object_pose_sub = rospy.Subscriber("/Pick_Object_Command",Float32MultiArray,pick_object_callback)
    safety_radius=0.2
    goal = PoseStamped()

    pick_object_pose = Float32MultiArray()

    move_distance = 0.01
    getplan = False
    while not rospy.is_shutdown():

        # x = goal.pose.position.x
        # y = goal.pose.position.y
        #target_point = Point(x,y,0)

        if(len(pick_object_pose.data) != 0):
            current_pose = get_current_pose()
            finder = NearestSafePositionFinder(safety_radius)  # 安全半径

            rospy.sleep(1)  # 地図データの受信を待つ
            point_Before = Point(current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z)
            
            optimalPos = optimalPosition(current_pose,pick_object_pose)

            nearest_safe_position = finder.get_nearest_safe_position(optimalPos,point_Before,move_distance)
            if nearest_safe_position:
                print("Nearest safe position found at:", nearest_safe_position.x, nearest_safe_position.y)

            # plan = get_plan(0.0,0.0,1.25,0.0, 0.0)
            if(getplan == False):
                plan = get_plan(current_pose.pose.position.x,current_pose.pose.position.y,nearest_safe_position.x,nearest_safe_position.y, 0.0)
            if(len(plan.poses) != 0):
                getplan = True
                waypoint = process_path(plan)

                plan.poses = waypoint

                plan_pub.publish(plan)
                
                break
            
        rate.sleep()

# def get_plan(start, goal):
#     rospy.wait_for_service('move_base/GlobalPlanner/make_plan')
#     try:
#         # 経路計画サービスのクライアントを作成
#         get_plan = rospy.ServiceProxy('move_base/GlobalPlanner/make_plan', GetPlan)

#         # 経路計画リクエストを作成してサービスを呼び出し
#         plan = get_plan(start, goal, 0.0)  # 0.0は許容誤差
#         print("get plan")
#         # 経路を返す
#         return plan.plan
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s" % e)

# def get_plan(x_start, y_start, x_goal, y_goal, yaw_goal):
#     rospy.wait_for_service('make_plan')
#     try:
#         # 経路計画サービスのクライアントを作成
#         get_plan_service = rospy.ServiceProxy('make_plan', GetPlan)

#         # 開始位置を設定
#         start = PoseStamped()
#         start.header.frame_id = "map"
#         start.pose.position.x = x_start
#         start.pose.position.y = y_start
#         # 開始位置の向きは任意に設定可能
#         start.pose.orientation.w = 1.0  # 北を向く

#         # 目標位置を設定
#         goal = PoseStamped()
#         goal.header.frame_id = "map"
#         goal.pose.position.x = x_goal
#         goal.pose.position.y = y_goal
#         # 目標位置の向きを設定
#         q = quaternion_from_euler(0, 0, yaw_goal)
#         goal.pose.orientation.x = q[0]
#         goal.pose.orientation.y = q[1]
#         goal.pose.orientation.z = q[2]
#         goal.pose.orientation.w = q[3]

#         # 経路計画リクエストを作成してサービスを呼び出し
#         plan = get_plan_service(start, goal, 0.0)  # 0.0は許容誤差

#         # 経路を返す
#         return plan.plan
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s" % e)

# # if __name__ == '__main__':
# #     rospy.init_node('get_path_node', anonymous=True)
# #     threshold = 1
    
# #     pose_sub = rospy.Subscriber("Base_Move",PoseStamped,move_callback)

# #     goal = PoseStamped()

# #     current_pose = get_current_pose()
# #     print("start plan get")
# #     try:
# #         plan = get_plan(0, 0, 1, 1, 0)
# #         # plan = get_plan(current_pose,goal)
# #         print(plan)  # 経路をコンソールに表示
# #     except rospy.ROSInterruptException:
# #         print("not plan")
# #         pass

# #     rospy.spin()

# rospy.init_node('get_path_node', anonymous=True)
# plan = get_plan(0, 0, 1, 1, 0)

# rospy.spin()