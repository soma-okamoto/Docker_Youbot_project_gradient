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
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import numpy as np
from tf import TransformListener
from geometry_msgs.msg import PointStamped,PoseStamped, Quaternion
import signal
import sys
import tf.transformations as tf_trans

def get_current_pose():
    # 現在のロボットの位置を取得する
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    trans = tf_buffer.lookup_transform('map', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
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


def align_pointclouds(pointclouds):
    # 最初のポイントクラウドをターゲットとして設定
    target = pointclouds[0]
    # 法線推定
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    
    # ICPのパラメータを設定
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100, 
                                                                relative_fitness=1e-6,
                                                                relative_rmse=1e-6)

    # 他のポイントクラウドを順番にターゲットに合わせていく
    for i in range(1, len(pointclouds)):
        source = pointclouds[i]

        # ICPアルゴリズムを使用してポイントクラウドを合わせる
        icp_result = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance=0.05,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=criteria)  # 収束条件を設定

        # 変換行列を適用してソースポイントクラウドを合わせる
        source.transform(icp_result.transformation)

        # 合わせたポイントクラウドを新たなターゲットに設定
        target += source

    return target

def merge_similar_points(pointcloud, threshold):
    # KDツリーを構築して最近傍点を検索
    pcd_tree = o3d.geometry.KDTreeFlann(pointcloud)

    merged_points = []
    visited = np.zeros(len(pointcloud.points), dtype=bool)

    for i in range(len(pointcloud.points)):
        if visited[i]:
            continue

        indices = []
        query_result = pcd_tree.search_radius_vector_3d(pointcloud.points[i], threshold)

        for index in query_result[1]:
            if not visited[index]:
                visited[index] = True
                indices.append(index)

        if indices:
            merged_point = np.mean(np.asarray(pointcloud.points)[indices], axis=0)
            merged_points.append(merged_point)
    
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd.points = o3d.utility.Vector3dVector(np.asarray(merged_points))

    return merged_pcd

def downsample_callback(data):

    global listener, accumulated_pc
    print("Save Start")

    # PointCloud2からOpen3DのPointCloudに変換
    pc = point_cloud2.read_points(data, skip_nans=True)
    points = []

    camera_pos = np.array([0,0,0])


    for p in pc:
        point = np.array([p[0], p[1], p[2]])

        # カメラ位置からの距離がしきい値以上の点のみを抽出する
        threshold = 2
        if np.linalg.norm(point - camera_pos) > threshold:
            continue

        points.append(point)

    # Open3DのPointCloudに変換
    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(points)

    # VoxelDownSampleでダウンサンプリング
    # voxel_size = 0.001
    # pc_o3d_downsampled = pc_o3d.voxel_down_sample(voxel_size)

    # ポイントクラウドを結合
    accumulated_pc.append(pc_o3d)
    print("Save and publish PointCloud!!")
    pc_sub.unregister()

if __name__ == '__main__':
    x = 0
    y = 0
    theta = math.pi/6

    accumulated_pc = []

    # ノードを初期化する
    rospy.init_node('move_base_test', anonymous=True)


    while not rospy.is_shutdown():
    
        # ノードを初期化する
        rospy.init_node('move_base_test', anonymous=True)

        # 初期位置から半径1mの円周上の目標位置を設定する
        num_points = 12
        radius = 0.5
        

        # TransformListenerを初期化
        listener = TransformListener()
        

        result = move_base_client(x,y, theta)

        if result:
            # Subscriberの設定
            pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, downsample_callback)

            rospy.loginfo("Goal execution Done")
            rospy.loginfo("x  : " + str(x))

            x += 0.2
            y = 0.2
            theta =0
            result = None
            rospy.sleep(1)

        if (x >=1.0 and x <=1.4):
            y = 0
            theta = - math.pi/6
        
        if (x >=1.4 and x <=1.6):
            y = -0.2
            theta = - math.pi/3

        if (x >=1.6):

            # 類似点をマージする閾値
            threshold = 1

            # ポイントクラウドを合わせる
            aligned_pointcloud = align_pointclouds(accumulated_pc)

            #merged_points = merge_similar_points(accumulated_pc, aligned_pointcloud)
            print("saving")
            output_path = './Save_move_pointcloud-icp.ply'
            o3d.io.write_point_cloud(output_path, aligned_pointcloud)
            break
            
        

    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation test finished.")
    # rospy.spin()

