#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import heapq
import tf2_ros
import tf2_geometry_msgs

# グローバル変数
cost_map = None
raw_grid = None
map_resolution = None
map_origin_x = 0
map_origin_y = 0
map_width = 0
map_height = 0

# RViz用パブリッシャ
path_array_pub = None

def map_callback(msg):
    global raw_grid, cost_map, map_resolution, map_origin_x, map_origin_y, map_width, map_height

    # SLAM Toolbox からの OccupancyGrid メッセージを取得
    raw_grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    map_resolution = msg.info.resolution
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y
    map_width = msg.info.width
    map_height = msg.info.height

    # コストマップを生成
    cost_map = create_cost_map(raw_grid)

    rospy.loginfo("Map received and cost map created.")

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
                                    cost_map[ni][nj] = 100
    return cost_map

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def expand_obstacles(cost_map, robot_radius):
    """
    ロボットの大きさを考慮し、占有セルを膨張させる。
    robot_radius: ロボットの半径 (メートル)
    """
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
                                expanded_map[ni, nj] = 100
    return expanded_map

def astar_with_cost_map(cost_map, start, goal):
    rows = len(cost_map)
    cols = len(cost_map[0])

    open_list = []
    closed_set = set()

    start_g = cost_map[start[0], start[1]]
    start_f = start_g + heuristic(start, goal)
    heapq.heappush(open_list, (start_f, start_g, start, None))

    g_scores = {start: start_g}
    came_from = {}

    directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]

    while open_list:
        current_f, current_g, current_node, parent_node = heapq.heappop(open_list)
        if current_node in closed_set:
            continue

        closed_set.add(current_node)
        came_from[current_node] = parent_node

        if current_node == goal:
            return reconstruct_path(came_from, current_node)

        cx, cy = current_node

        for dx, dy in directions:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < rows and 0 <= ny < cols):
                continue
            if cost_map[nx, ny] == 100:
                continue

            new_g = current_g + (1.414 if dx != 0 and dy != 0 else 1)
            if (nx, ny) in closed_set:
                if new_g >= g_scores.get((nx, ny), float('inf')):
                    continue
                closed_set.remove((nx, ny))

            if new_g < g_scores.get((nx, ny), float('inf')):
                g_scores[(nx, ny)] = new_g
                f_score = new_g + heuristic((nx, ny), goal)
                heapq.heappush(open_list, (f_score, new_g, (nx, ny), current_node))

    return None

def reconstruct_path(came_from, current):
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def get_robot_position():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(3.0))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        return (x, y)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get robot position.")
        return None

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

def goal_callback(msg):
    global raw_grid, cost_map

    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y

    goal_map_x = int((goal_x - map_origin_x) / map_resolution)
    goal_map_y = int((goal_y - map_origin_y) / map_resolution)

    robot_position = get_robot_position()
    if robot_position is None:
        rospy.logwarn("Robot position not available. Skipping path planning.")
        return

    start_x = int((robot_position[0] - map_origin_x) / map_resolution)
    start_y = int((robot_position[1] - map_origin_y) / map_resolution)

    start = (start_y, start_x)
    goal = (goal_map_y, goal_map_x)

    if not (0 <= goal[0] < cost_map.shape[0] and 0 <= goal[1] < cost_map.shape[1]):
        rospy.logwarn("Goal is out of map bounds!")
        return

    expanded_cost_map = expand_obstacles(cost_map, robot_radius=0.3)  # ロボット半径を0.3mに設定

    path = astar_with_cost_map(expanded_cost_map, start, goal)

    if path:
        rospy.loginfo("Path found!")
        publish_path_rviz(path, start, goal)
    else:
        rospy.logwarn("No path found!")

if __name__ == "__main__":
    rospy.init_node('path_planner', anonymous=True)

    path_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    rospy.spin()
