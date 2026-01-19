#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import numpy as np
import heapq
import tf2_ros
import tf2_geometry_msgs
import math
from tf.transformations import euler_from_quaternion,quaternion_from_euler

# ---------------------------
# グローバル変数
# ---------------------------
cost_map = None
raw_grid = None
map_resolution = None
map_origin_x = 0
map_origin_y = 0
map_width = 0
map_height = 0

initialize_Robot_Pos = None
velocity_publisher = None
gool_publisher = None
path_array_pub = None
path_penalty_pub = None

# PENALTY_COST（膨張コスト）などもグローバル定義
PENALTY_COST = 50

tf_buffer = None
tf_listener = None

def map_callback(msg):
    global raw_grid, cost_map, map_resolution, map_origin_x, map_origin_y, map_width, map_height
    raw_grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    map_resolution = msg.info.resolution
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y
    map_width = msg.info.width
    map_height = msg.info.height
    cost_map = create_cost_map(raw_grid)

def create_cost_map(raw_grid, avoid_distance=1):
    rows = len(raw_grid)
    cols = len(raw_grid[0])
    cost_map = np.zeros((rows, cols))
    
    for i in range(rows):
        for j in range(cols):
            value = raw_grid[i][j]
            if value == 100:
                cost_map[i][j] = 100  # 占有セル
            elif value == -1:
                cost_map[i][j] = 30   # 未知セル
            else:
                cost_map[i][j] = 0    # 空きセル

    # 占有セル近傍を膨張
    for i in range(rows):
        for j in range(cols):
            if raw_grid[i][j] == 100:
                for di in range(-avoid_distance, avoid_distance+1):
                    for dj in range(-avoid_distance, avoid_distance+1):
                        ni = i + di
                        nj = j + dj
                        if 0 <= ni < rows and 0 <= nj < cols:
                            if abs(di) + abs(dj) <= avoid_distance:
                                if raw_grid[ni][nj] != 100:
                                    cost_map[ni][nj] = PENALTY_COST
    return cost_map

def expand_obstacles(cost_map, robot_radius):
    rows, cols = cost_map.shape
    expanded_map = np.copy(cost_map)
    cell_radius = int(np.ceil(robot_radius / map_resolution))

    for i in range(rows):
        for j in range(cols):
            if cost_map[i, j] == 100:
                for di in range(-cell_radius, cell_radius + 1):
                    for dj in range(-cell_radius, cell_radius + 1):
                        ni = i + di
                        nj = j + dj
                        if 0 <= ni < rows and 0 <= nj < cols:
                            if np.sqrt(di**2 + dj**2) <= cell_radius:
                                expanded_map[ni, nj] = PENALTY_COST
    return expanded_map

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar_with_cost_map(cost_map, start, goal):
    rows = len(cost_map)
    cols = len(cost_map[0])

    open_list = []
    closed_set = set()

    start_g = 0
    start_f = start_g + heuristic(start, goal)
    heapq.heappush(open_list, (start_f, start_g, start, None))

    g_scores = {start: start_g}
    came_from = {}

    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while open_list:
        current_f, current_g, current_node, parent_node = heapq.heappop(open_list)
        if current_node in closed_set:
            continue

        closed_set.add(current_node)
        came_from[current_node] = parent_node

        if current_node == goal:
            return reconstruct_path(came_from, current_node), current_g

        cx, cy = current_node

        for dx, dy in directions:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < rows and 0 <= ny < cols):
                continue
            if cost_map[nx, ny] == 100:
                continue

            # cost_map[nx,ny] が PENALTY_COST(=50) なら移動コストを上げる
            move_cost = 1 + (PENALTY_COST if cost_map[nx, ny] == PENALTY_COST else 0)
            new_g = current_g + move_cost

            if (nx, ny) in closed_set:
                if new_g >= g_scores.get((nx, ny), float('inf')):
                    continue
                closed_set.remove((nx, ny))

            if new_g < g_scores.get((nx, ny), float('inf')):
                g_scores[(nx, ny)] = new_g
                f_score = new_g + heuristic((nx, ny), goal)
                heapq.heappush(open_list, (f_score, new_g, (nx, ny), current_node))

    return None, float('inf')

def reconstruct_path(came_from, current):
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def publish_path_rviz(path, start, goal):
    marker_array = MarkerArray()

    path_marker = Marker()
    path_marker.header.frame_id = "map"
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "path"
    path_marker.id = 0
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.scale.x = 0.05
    path_marker.color.a = 1.0
    path_marker.color.r = 0.0
    path_marker.color.g = 0.0
    path_marker.color.b = 1.0

    for point in path:
        p = Point()
        p.x = point[1] * map_resolution + map_origin_x
        p.y = point[0] * map_resolution + map_origin_y
        p.z = 0.0
        path_marker.points.append(p)
    marker_array.markers.append(path_marker)

    goal_marker = Marker()
    goal_marker.header.frame_id = "map"
    goal_marker.header.stamp = rospy.Time.now()
    goal_marker.ns = "goal"
    goal_marker.id = 1
    goal_marker.type = Marker.SPHERE
    goal_marker.action = Marker.ADD
    goal_marker.scale.x = 0.2
    goal_marker.scale.y = 0.2
    goal_marker.scale.z = 0.2
    goal_marker.color.a = 1.0
    goal_marker.color.r = 1.0
    goal_marker.color.g = 0.0
    goal_marker.color.b = 0.0
    goal_marker.pose.position.x = goal[1] * map_resolution + map_origin_x
    goal_marker.pose.position.y = goal[0] * map_resolution + map_origin_y
    goal_marker.pose.position.z = 0.0
    goal_marker.pose.orientation.w = 1.0
    marker_array.markers.append(goal_marker)

    path_array_pub.publish(marker_array)

def generate_waypoints(path, interval):
    waypoints = []
    distance = 0
    waypoints.append(path[0])

    for i in range(1, len(path)):
        prev = path[i - 1]
        curr = path[i]

        if (curr[0] != prev[0] and curr[1] == prev[1]) or (curr[1] != prev[1] and curr[0] == prev[0]):
            waypoints.append(prev)

        dx = (curr[1] - prev[1]) * map_resolution
        dy = (curr[0] - prev[0]) * map_resolution
        distance += math.sqrt(dx**2 + dy**2)

        if distance >= interval:
            waypoints.append(curr)
            distance = 0

    waypoints.append(path[-1])
    return waypoints

def move_along_waypoints(waypoints, goal_theta ,status):
    global result, velocity_publisher
    for i,waypoint in enumerate(waypoints):
        while True:
            twist = Twist()

            # --- ロボットの現在位置を取得 ---
            robot_position = get_robot_position()
            if robot_position is None:
                rospy.logwarn("Failed to get robot position. Aborting movement.")
                return

            current_orientation = get_robot_orientation()
            if current_orientation is None:
                current_orientation = 0.0

            # --- 目標位置 ---
            target_x = waypoint[1] * map_resolution + map_origin_x
            target_y = waypoint[0] * map_resolution + map_origin_y

            dx = target_x - robot_position[0]
            dy = target_y - robot_position[1]
            distance = math.sqrt(dx**2 + dy**2)

            # ウェイポイント到達判定
            if distance < 0.1:  # 10cm以内
                twist = Twist()
                velocity_publisher.publish(twist)
                break

            # --- ロボット座標系への変換 ---
            cos_th = math.cos(-current_orientation)
            sin_th = math.sin(-current_orientation)
            local_x = dx * cos_th - dy * sin_th
            local_y = dx * sin_th + dy * cos_th

            twist.linear.x = local_x * 0.3
            twist.linear.y = local_y * 0.3

            velocity_publisher.publish(twist)
            rospy.sleep(0.1)

    rotate_to_orientation(goal_theta)
    # 停止コマンド
    twist = Twist()
    velocity_publisher.publish(twist)
    result = True

def get_robot_position():
    """map->base_linkのtransformを使い、位置(x,y)を返す。"""
    global tf_buffer
    try:

        trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        return (x, y)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get robot position.")

        twist = Twist()
        velocity_publisher.publish(twist)
        return None

def get_robot_orientation():

    global tf_buffer
    try:
        trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
        orientation_q = trans.transform.rotation
        euler = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return euler[2]  # yaw
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get robot orientation.")
        return None
# -----------------------------------------

def rotate_to_orientation(target_theta):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_theta = get_robot_orientation()
        if current_theta is None:
            rospy.logwarn("Failed to get robot orientation.")
            return
        error = target_theta - current_theta
        # 誤差を[-pi, pi]に正規化
        error = math.atan2(math.sin(error), math.cos(error))
        if abs(error) < 0.05:
            break
        twist = Twist()
        twist.angular.z = 0.3 if error > 0 else -0.3
        velocity_publisher.publish(twist)
        rate.sleep()

def set_initial_robot_position():
    global initialize_Robot_Pos
    if initialize_Robot_Pos is None:
        robot_position = get_robot_position()
        if robot_position is not None:
            initialize_Robot_Pos = robot_position
            rospy.loginfo(f"Initial robot position set to: {initialize_Robot_Pos}")
        else:
            rospy.logwarn("Failed to set initial robot position. Retrying...")
    else:
        rospy.loginfo(f"Initial robot position already set: {initialize_Robot_Pos}")

def publish_path_as_path_message(path_with_penalty, interval=0.5):
    global initialize_Robot_Pos

    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped

    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"

    if initialize_Robot_Pos is None:
        rospy.logwarn("Initial robot position is not set. Cannot publish path.")
        return

    sampled_path = []
    sampled_path.append(path_with_penalty[0])
    accumulated_distance = 0.0

    for i in range(1, len(path_with_penalty)):
        prev = path_with_penalty[i - 1]
        curr = path_with_penalty[i]
        dx = (curr[1] - prev[1]) * map_resolution
        dy = (curr[0] - prev[0]) * map_resolution
        distance = math.sqrt(dx**2 + dy**2)
        accumulated_distance += distance
        if accumulated_distance >= interval:
            sampled_path.append(curr)
            accumulated_distance = 0.0

    if sampled_path[-1] != path_with_penalty[-1]:
        sampled_path.append(path_with_penalty[-1])

    for i, point in enumerate(sampled_path):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = point[1] * map_resolution + map_origin_x - initialize_Robot_Pos[0]
            pose_stamped.pose.position.y = point[0] * map_resolution + map_origin_y - initialize_Robot_Pos[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            # # 向きを計算
            # if i > 0:
            #     prev_point = sampled_path[i - 1]
            #     dx = (point[1] - prev_point[1]) * map_resolution
            #     dy = (point[0] - prev_point[0]) * map_resolution
            #     yaw = math.atan2(dy, dx)
            # else:
            #     yaw = 0.0

            # # ヨー角をクォータニオンに変換
            # quaternion = quaternion_from_euler(0, 0, yaw)
            # pose_stamped.pose.orientation.x = quaternion[0]
            # pose_stamped.pose.orientation.y = quaternion[1]
            # pose_stamped.pose.orientation.z = quaternion[2]
            # pose_stamped.pose.orientation.w = quaternion[3]

            path_msg.poses.append(pose_stamped)

    path_penalty_pub.publish(path_msg)
    rospy.loginfo("Published sampled path_with_penalty as Path message.")

def route_callback(msg):
    global result
    generate_route = msg

    if generate_route is not None:
        first_path = generate_route.poses
        for i in range(len(first_path)):
            result = False
            goal_x = first_path[i].pose.position.x + initialize_Robot_Pos[0]
            goal_y = first_path[i].pose.position.y + initialize_Robot_Pos[1]

            goal_orientation = first_path[i].pose.orientation
            euler_angles = euler_from_quaternion([
                goal_orientation.x,
                goal_orientation.y,
                goal_orientation.z,
                goal_orientation.w
            ])
            goal_theta = euler_angles[2]

            goal_map_x = int((goal_x - map_origin_x) / map_resolution)
            goal_map_y = int((goal_y - map_origin_y) / map_resolution)

            # 現在位置をマップ座標に
            robot_position = get_robot_position()
            if robot_position is None:
                rospy.logwarn("Cannot get current robot position.")
                return
            start_x = int((robot_position[0] - map_origin_x) / map_resolution)
            start_y = int((robot_position[1] - map_origin_y) / map_resolution)

            start = (start_y, start_x)
            goal = (goal_map_y, goal_map_x)

            expanded_cost_map = expand_obstacles(cost_map, robot_radius=0.4)
            path_with_penalty, cost_with_penalty = astar_with_cost_map(expanded_cost_map, start, goal)

            no_penalty_map = np.where(expanded_cost_map == PENALTY_COST, 0, expanded_cost_map)
            path_without_penalty, cost_without_penalty = astar_with_cost_map(no_penalty_map, start, goal)

            route = path_without_penalty
            print(route)
            if path_without_penalty and cost_without_penalty < cost_with_penalty:
                rospy.loginfo("Using detour (with penalty).")
                route = path_with_penalty
                publish_path_as_path_message(route)
                rospy.sleep(8)

                waypoints = generate_waypoints(route, 0.5)
                publish_path_rviz(route, start, goal)
                move_along_waypoints(waypoints, goal_theta,"withPenarty")
            else:
                rospy.loginfo("Using shortest path (no penalty).")
                route = path_without_penalty

                waypoints = generate_waypoints(route, 0.5)
                publish_path_rviz(route, start, goal)
                move_along_waypoints(waypoints, goal_theta,"withoutPenarty")

            while not rospy.is_shutdown():
                rospy.sleep(1.0)
                if result:
                    result = False
                    break

        # 全ウェイポイント到達したらゴールステータスを出す
        gool_status = True
        gool_publisher.publish(gool_status)
        gool_status = False

def main():
    global velocity_publisher
    global path_array_pub, path_penalty_pub
    global gool_publisher
    global tf_buffer, tf_listener

    rospy.init_node('path_planner', anonymous=True)

    # ---- ここで一度だけ作成して使い回す ----
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # ---------------------------------------

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    path_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    path_penalty_pub = rospy.Publisher('/path_with_penalty', Path, queue_size=10)
    gool_publisher = rospy.Publisher('gool_state', Bool, queue_size=1)

    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/generate_route", Path, route_callback)

    # tf情報が来るまで待って初期位置設定
    while initialize_Robot_Pos is None and not rospy.is_shutdown():
        set_initial_robot_position()
        rospy.sleep(1.0)

    rospy.spin()

if __name__ == "__main__":
    main()
