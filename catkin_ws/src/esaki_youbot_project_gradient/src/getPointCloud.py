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

import time

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


def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size,source,target):
    # print(":: Load two point clouds and disturb initial pose.")

    # source = o3d.io.read_point_cloud("0.ply")
    # target = o3d.io.read_point_cloud("10.ply")
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    # draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh



def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result



def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size,result_ransac):
    distance_threshold = voxel_size * 0.4
    # print(":: Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result



def downsample_callback(data):

    global listener, accumulated_pc,target,source
    print("Save Start")
    registration = False

    
    print("registration : " + str(registration))
    if(target == None):
        print("target save")
        pc = point_cloud2.read_points(data,skip_nans=True) 


        points = []

        camera_pos = np.array([0,0,0])


        # 四元数の作成

        # 回転行列の作成

        for p in pc:
            point = np.array([p[0], p[1], p[2]])

            # カメラ位置からの距離がしきい値以上の点のみを抽出する
            threshold = 2
            if np.linalg.norm(point - camera_pos) > threshold:
                continue
            
            # # ベクトルの回転

            points.append(point)

        # Open3DのPointCloudに変換
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(points)
        target.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1,
            max_nn=30
        )
        )

    else:
        print("source Save")
        pc = point_cloud2.read_points(data,skip_nans=True)

        points = []

        camera_pos = np.array([0,0,0])

        for p in pc:
            point = np.array([p[0], p[1], p[2]])

            # カメラ位置からの距離がしきい値以上の点のみを抽出する
            threshold = 2
            if np.linalg.norm(point - camera_pos) > threshold:
                continue
            points.append(point)
        print("get Pointcloud")
        # Open3DのPointCloudに変換
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(points)
        source.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1,
            max_nn=30
        )
        )

        
        voxel_size = 0.05  # means 5cm for this dataset
        source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
            voxel_size,source,target)

        result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
        print("execute_global_registration")
        result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                        voxel_size,result_ransac)
        print("refine_registration")
        source.transform(result_icp.transformation)

        target += source

        print(len(target.points))

        # Open3DのPointCloudからPointCloud2に変換
        header = Header()
        header.frame_id = data.header.frame_id

        # # VoxelDownSampleでダウンサンプリング
        voxel_size = 0.01
        pc_o3d_downsampled = target.voxel_down_sample(voxel_size)

        downsampled_points = pc_o3d_downsampled.points
        downsampled_pc2 = point_cloud2.create_cloud_xyz32(header, downsampled_points)

        # ダウンサンプリングした点群をPublish
        #downsampled_pub.publish(downsampled_pc2)
        print("downsample publish")
            

    pc_sub.unregister()

if __name__ == '__main__':
    x = 0
    y = 0
    # theta = math.pi/6
    theta = 0

    accumulated_pc = []

    # ノードを初期化する
    rospy.init_node('move_base_test', anonymous=True)

    target = None
    source = None

    cmd = 0

    while not rospy.is_shutdown():
    
        # # ノードを初期化する
        # rospy.init_node('move_base_test', anonymous=True)
        # Publisherの設定
        downsampled_pub = rospy.Publisher('downsampled_pc', PointCloud2, queue_size=1)

        # 初期位置から半径1mの円周上の目標位置を設定する
        num_points = 12
        radius = 0.5
        
        # TransformListenerを初期化
        listener = TransformListener()

        time.sleep(5)
        cmd += 1

        # Subscriberの設定
        pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, downsample_callback)

        print("registration")
        rospy.loginfo("Goal execution Done")
        rospy.loginfo("x  : " + str(x))

        x += 0.2
        y = 0.0
        theta =0
        result = None
        #rospy.sleep(1)
        # break

        if (x >=0.6):
            
            rospy.sleep(20)
            # 類似点をマージする閾値
            threshold = 1

            # ポイントクラウドを合わせる
            #aligned_pointcloud = align_pointclouds(accumulated_pc)

            #merged_points = merge_similar_points(accumulated_pc, aligned_pointcloud)
            print("saving")
            output_path = './Save_move_pointcloud-global-registration6.ply'
            o3d.io.write_point_cloud(output_path, target)
            break

    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation test finished.")
    # rospy.spin()