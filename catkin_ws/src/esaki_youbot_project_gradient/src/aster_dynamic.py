#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import Header,String,Bool
import numpy as np
import heapq
import tf2_ros
import tf2_geometry_msgs
import math
from tf.transformations import euler_from_quaternion

# グローバル変数
cost_map = None
expanded_map = None
raw_grid = None
map_resolution = None
map_origin_x = 0
map_origin_y = 0
map_width = 0
map_height = 0

initialize_Robot_Pos = None
# RViz用パブリッシャ
velocity_publisher = None
gool_publisher = None

PENALTY_COST = 50  # 50のグリッドのペナルティ

# RViz用パブリッシャ
path_array_pub = None
path_penalty_pub = None

ORIENTATION_INC = math.pi / 6
MOVEMENTS = {
    (1,0,0),
    (-1,0,0),
    (0,1,0),
    (0,-1,0),
    (0,0,1),
    (0,0,-1)
}

robot_width = 0.6
robot_height = 0.4

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
        pose = Point()
        pose.x = point[1] * map_resolution + map_origin_x
        pose.y = point[0] * map_resolution + map_origin_y
        pose.z = 0.0
        path_marker.points.append(pose)
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

def map_callback(msg):
    global raw_grid, cost_map,expanded_map, map_resolution, map_origin_x, map_origin_y, map_width, map_height

    # SLAM Toolbox からの OccupancyGrid メッセージを取得
    raw_grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    map_resolution = msg.info.resolution
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y
    map_width = msg.info.width
    map_height = msg.info.height

    cost_map = create_cost_map(raw_grid)


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


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

def dynamic_expand_obstacles(cost_map, robot_width,robot_height,direction):
    rows, cols = cost_map.shape
    expanded_map = np.copy(cost_map)

    if direction == "forward":
        cell_radius = int(math.ceil(robot_height/map_resolution))
    elif direction == "back":
        cell_radius = int(math.ceil(robot_height/2 * map_resolution))
    elif direction in ["left","right"]:
        cell_radius = int(math.ceil(robot_width / map_resolution))
    else:
        cell_radius = 1

    for i in range(rows):
        for j in range(cols):
            if cost_map[i,j] == 100:
                for di in range(-cell_radius,cell_radius + 1):
                    for dj in range(-cell_radius,cell_radius + 1):
                        ni = i + di
                        nj = j + dj
                        if 0 <= ni < rows and 0 <= nj < cols:
                            if cost_map[ni][nj] != 100:
                                cost_map[ni][nj] = PENALTY_COST
    return expanded_map

def get_movement_direction(prev_state,current_state):
    if prev_state is None:
        return "forward"
    (px,py,pth) = prev_state
    (cx,cy,cth) = current_state

    dx = cx - px
    dy = cy - py
    
    if abs(dx) > abs(dy):
        return "forward" if dx > 0 else "backward"
    elif abs(dy) > abs(dx):
        return "right" if dy > 0 else "left"
    else:
        return "rotate"




def astar_with_dynamic_obstacles(cost_map, start, goal,goal_theta):
    rows = len(cost_map)
    cols = len(cost_map[0])

    open_list = []
    closed_set = set()

    start_state = (start[0],start[1],0)
    start_g = 0
    start_f = start_g + heuristic(start, goal)
    heapq.heappush(open_list, (start_f, start_g, start_state, None))

    g_scores = {start: start_g}
    came_from = {}

    predirection = None

    while open_list:
        current_f, current_g, current_state, parent_state = heapq.heappop(open_list)
        if current_state in closed_set:
            continue

        closed_set.add(current_state)
        came_from[current_state] = parent_state

        if (current_state[0],current_state[1]) == goal:
            return reconstruct_path(came_from, current_state), current_g

        direction = get_movement_direction(parent_state,current_state)
        
        if predirection == None or direction != predirection:
            print(direction)
            predirection = direction
            expanded_map = dynamic_expand_obstacles(cost_map,robot_width,robot_height,direction)

        for dx, dy,dtheta in MOVEMENTS:
            new_x = current_state[0] + dx
            new_y = current_state[1] + dy
            new_theta = (current_state[2] + dtheta) % (2 * math.pi)
            if not (0 <= new_x < rows and 0 <= new_y < cols):
                continue

            if expanded_map[new_x,new_y] == 100:
                continue

            move_cost = 1 + (PENALTY_COST if expanded_map[new_x,new_y] == PENALTY_COST else 0)
            new_g = current_g + move_cost
            new_state = (new_x,new_y,new_theta)

            if new_state in closed_set:
                if new_g >= g_scores.get(new_state,float("inf")):
                    continue
                closed_set.remove(new_state)

            if new_g < g_scores.get(new_state, float("inf")):
                g_scores[new_state] = new_g
                f_score = new_g + heuristic((new_x,new_y),goal)
                heapq.heappush(open_list,(f_score,new_g,new_state,current_state))

    return None, float('inf')

def set_initial_robot_position():
    global initialize_Robot_Pos
    # 初期位置が未設定の場合に取得
    if initialize_Robot_Pos is None:
        robot_position = get_robot_position()
        if robot_position is not None:
            initialize_Robot_Pos = robot_position
            rospy.loginfo(f"Initial robot position set to: {initialize_Robot_Pos}")
        else:
            rospy.logwarn("Failed to set initial robot position. Retrying...")
    else:
        rospy.loginfo(f"Initial robot position already set: {initialize_Robot_Pos}")

def get_robot_position():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    return (x, y)


def get_robot_orientation():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(3.0))
        orientation_q = trans.transform.rotation
        euler = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        return euler[2]  # Yaw
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get robot orientation.")
        return None

def rotate_to_orientation(target_theta):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_theta = get_robot_orientation()
        if current_theta is None:
            rospy.logwarn("Failed to get robot orientation.")
            return
        error = target_theta - current_theta
        error = math.atan2(math.sin(error),math.cos(error))
        if abs(error) < 0.05:
            break
        twist = Twist()
        twist.angular.z = 0.3 if error > 0 else -0.3
        velocity_publisher.publish(twist)
        rate.sleep()

def generate_waypoints(path, interval):
    waypoints = []
    distance = 0
    waypoints.append(path[0])

    for i in range(1, len(path)):
        prev = path[i - 1]
        curr = path[i]

        # 曲がり角の判定
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

def move_along_waypoints_dynamic(waypoints,final_theta):
    global result,velocity_publisher

    for waypoint in waypoints:
        while True:
            twist = Twist()

            # ロボットの現在位置を取得
            robot_position = get_robot_position()
            if robot_position is None:
                rospy.logwarn("Failed to get robot position. Aborting movement.")
                return

            # 目標位置の計算
            target_x = waypoint[1] * map_resolution + map_origin_x
            target_y = waypoint[0] * map_resolution + map_origin_y

            # 制御指令を計算
            dx = target_x - robot_position[0]
            dy = target_y - robot_position[1]
            distance = math.sqrt(dx**2 + dy**2)

            # ウェイポイント到達判定
            if distance < 0.1:  # 10cm以内で到達とみなす
                break
            angle_to_target = math.atan2(dy,dx)
            current_orientation = get_robot_orientation()

            if current_orientation is None:
                current_orientation = 0.0

            angular_error = angle_to_target - current_orientation
            angular_error = math.atan2(math.sin(angular_error),math.cos(angular_error))

            twist.linear.x = dx * 0.3
            twist.linear.y = dy * 0.3
            twist.angular.z = 0.5 * angular_error
            # twist.linear.y = dy * 0.5  # y方向の速度

            velocity_publisher.publish(twist)
            rospy.sleep(0.1)

    rotate_to_orientation(final_theta)
    # 停止コマンド
    twist = Twist()
    velocity_publisher.publish(twist)
    result = True


def publish_path_as_path_message(path_with_penalty, interval=0.5):
    global initialize_Robot_Pos

    # Path メッセージを作成
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"

    # 初期位置が設定されていることを確認
    if initialize_Robot_Pos is None:
        rospy.logwarn("Initial robot position is not set. Cannot publish path.")
        return

    # サンプリングされた経路を格納
    sampled_path = []
    sampled_path.append(path_with_penalty[0])  # 最初の点を追加
    accumulated_distance = 0.0

    for i in range(1, len(path_with_penalty)):
        prev = path_with_penalty[i - 1]
        curr = path_with_penalty[i]

        # 距離を計算
        dx = (curr[1] - prev[1]) * map_resolution
        dy = (curr[0] - prev[0]) * map_resolution
        distance = math.sqrt(dx**2 + dy**2)
        accumulated_distance += distance

        # 一定の間隔でポイントを追加
        if accumulated_distance >= interval:
            sampled_path.append(curr)
            accumulated_distance = 0.0

    # 最後の点を追加
    if sampled_path[-1] != path_with_penalty[-1]:
        sampled_path.append(path_with_penalty[-1])

    # サンプリングされたポイントを Path メッセージに変換
    for point in sampled_path:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        
        # グリッド座標をマップ座標に変換し、初期位置を考慮
        pose_stamped.pose.position.x = point[1] * map_resolution + map_origin_x - initialize_Robot_Pos[0]
        pose_stamped.pose.position.y = point[0] * map_resolution + map_origin_y - initialize_Robot_Pos[1]
        pose_stamped.pose.position.z = 0.0

        # Orientationは使用しないので初期値
        pose_stamped.pose.orientation.w = 1.0

        path_msg.poses.append(pose_stamped)

    # パブリッシュ
    path_penalty_pub.publish(path_msg)
    rospy.loginfo("Published sampled path_with_penalty as Path message with interval: {:.2f} m.".format(interval))


def route_callback(msg):
    global generate_route, result
    generate_route = msg

    if generate_route != None:
        first_path = generate_route.poses
        for i in range(len(first_path)):
            result = False
            goal_x = first_path[i].pose.position.x + initialize_Robot_Pos[0]
            goal_y = first_path[i].pose.position.y + initialize_Robot_Pos[1]

            goal_orientation = first_path[i].pose.orientation

            euler = euler_from_quaternion([
                goal_orientation.x,
                goal_orientation.y,
                goal_orientation.z,
                goal_orientation.w
            ])

            goal_theta = euler[2]
            print(goal_theta)
            goal_map_x = int((goal_x - map_origin_x) / map_resolution)
            goal_map_y = int((goal_y - map_origin_y) / map_resolution)

            robot_position = get_robot_position()

            if robot_position is None:
                rospy.logwarn("Failed to get robot position. Aborting route planning.")
            
            start_x = int((robot_position[0] - map_origin_x) / map_resolution)
            start_y = int((robot_position[1] - map_origin_y) / map_resolution)
            
            start = (start_y, start_x)
            goal = (goal_map_y, goal_map_x)
            
            # 50を含むルート
            path_with_penalty, cost_with_penalty = astar_with_dynamic_obstacles(cost_map, start, goal,goal_theta)

            # 50を避けたルート
            no_penalty_map = np.where(cost_map == PENALTY_COST, 0, cost_map)
            path_without_penalty, cost_without_penalty = astar_with_dynamic_obstacles(no_penalty_map, start, goal,goal_theta)

            route = path_without_penalty

            if route and cost_without_penalty < cost_with_penalty:
                rospy.loginfo("Using detour to avoid penalty zones. Extra cost: {:.2f}".format(cost_with_penalty - cost_without_penalty))
                waypoints = generate_waypoints(path_with_penalty, 0.5)
                route = path_with_penalty
                publish_path_as_path_message(route)
                rospy.sleep(5)

            else:
                rospy.loginfo("Using shortest path with penalty zones.")
                waypoints = generate_waypoints(path_without_penalty, 0.5)
                route = path_without_penalty
            
            publish_path_rviz(route, start, goal)
            print(waypoints)

            move_along_waypoints_dynamic(waypoints,goal_theta)
            while True:
                rospy.sleep(1.0)
                print(result)
                if result:
                    result = False
                    break

        gool_status = True
        gool_publisher.publish(gool_status)
        gool_status = False
        # todo : youbot moved

if __name__ == "__main__":

    rospy.init_node('path_planner', anonymous=True)

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    path_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    path_penalty_pub = rospy.Publisher('/path_with_penalty', Path, queue_size=10)
    gool_publisher = rospy.Publisher('gool_state',Bool,queue_size=1)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/generate_route",Path,route_callback)

    while initialize_Robot_Pos is None and not rospy.is_shutdown():
        set_initial_robot_position()
        rospy.sleep(1.0)

    rospy.spin()