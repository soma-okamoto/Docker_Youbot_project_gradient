#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import numpy as np
from tf import TransformListener
from geometry_msgs.msg import PointStamped,PoseStamped, Quaternion
import signal
import sys
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf.transformations as tf_trans


def get_current_pose():
    # 現在のロボットの位置を取得する
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.1)
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

def downsample_callback(data):

    global listener, accumulated_pc
    
    current_pose = get_current_pose()

    # PointCloud2からOpen3DのPointCloudに変換
    pc = point_cloud2.read_points(data, skip_nans=True)
    points = []

    camera_pos = np.array([0,0,0])



    base_trans,rot = current_pose.pose.position,current_pose.pose.orientation

    print(current_pose)

    # 四元数の作成
    quaternion = [rot.x, rot.y, rot.z, rot.w]

    # 回転行列の作成
    rotation_matrix = tf_trans.quaternion_matrix(quaternion)[:3, :3]



    for p in pc:
        point = np.array([p[0], p[1], p[2]])

        # カメラ位置からの距離がしきい値以上の点のみを抽出する
        threshold = 1
        if np.linalg.norm(point - camera_pos) > threshold:
            continue

        # ベクトルの回転
        transformed_point = np.dot(rotation_matrix, point) + np.array([base_trans.x, base_trans.y, base_trans.z])

        # 変換したポイントを配列に追加
        points.append(transformed_point)

    
    # Open3DのPointCloudに変換
    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(points)

    # VoxelDownSampleでダウンサンプリング
    voxel_size = 0.001
    pc_o3d_downsampled = pc_o3d.voxel_down_sample(voxel_size)

    # ポイントクラウドを結合
    if accumulated_pc is None:
        accumulated_pc = pc_o3d_downsampled
    else:
        accumulated_pc += pc_o3d_downsampled

    # ダウンサンプリング後の点群データを保存
    output_path = './Save_pointcloud.ply'
    o3d.io.write_point_cloud(output_path, accumulated_pc)

    # Open3DのPointCloudからPointCloud2に変換
    header = Header()
    header.frame_id = data.header.frame_id
    downsampled_points = accumulated_pc.points

    downsampled_pc2 = point_cloud2.create_cloud_xyz32(header, downsampled_points)

    # ダウンサンプリングした点群をPublish
    downsampled_pub.publish(downsampled_pc2)
    print("Save and publish PointCloud!!")


def save_point_cloud(event):
    global accumulated_pc, save_count

    if accumulated_pc is not None:
        print("save pointcloud")
        # ポイントクラウドを保存する処理
        o3d.io.write_point_cloud(f"save_point_cloud_{save_count}.ply", accumulated_pc)
        save_count += 1
        print("save pointcloud  : " + str(save_count))


if __name__ == '__main__':

    rospy.init_node('downsample_node', anonymous=True)
    threshold = 1
    save_count = 0
    accumulated_pc = None

    # TransformListenerを初期化
    listener = TransformListener()

    # Subscriberの設定
    pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, downsample_callback)


    # Publisherの設定
    downsampled_pub = rospy.Publisher('downsampled_pc', PointCloud2, queue_size=1)

    # Timerを設定して1秒ごとにポイントクラウドを保存する
    #save_timer = rospy.Timer(rospy.Duration(0.1), save_point_cloud)

    # シグナルハンドラを登録する
    #signal.signal(signal.SIGINT, signal_handler)

    rospy.spin()
