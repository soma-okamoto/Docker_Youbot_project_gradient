#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import numpy as np
from tf import TransformListener
from geometry_msgs.msg import PointStamped


def downsample_callback(data):
    global listener
    
    
    # PointCloud2からOpen3DのPointCloudに変換
    pc = point_cloud2.read_points(data, skip_nans=True)
    points = []
    
    camera_pos = np.array([0,0,0])
    for p in pc:
        point = np.array([p[0],p[1],p[2]])

        # geometry_msgs/Point を使用する場合、以下のように変換する必要があります
        point_stamped = PointStamped()
        point_stamped.header = data.header  # ヘッダーを設定
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]
        point_stamped.header.stamp = rospy.Time(0)

        print("time" + str(rospy.Time))
        listener.waitForTransform("base_link", "camera_color_optical_frame", point_stamped.header.stamp, rospy.Duration(1.0))
        print("time" + str(rospy.Time))
        # base_link に変換
        point_transformed = listener.transformPoint("base_link", point_stamped)
        print("time" + str(rospy.Time))

        
        # カメラ位置からの距離がしきい値以上の点のみを抽出する
        camera_pos = np.array([0,0,0])
        threshold = 1
        if(np.linalg.norm(np.array([point_transformed.point.x, point_transformed.point.y, point_transformed.point.z]) - camera_pos) > threshold):
            continue
        
        # 変換したポイントを配列に追加
        points.append(np.array([point_transformed.point.x, point_transformed.point.y, point_transformed.point.z]))

    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(points)

    # VoxelDownSampleでダウンサンプリング
    voxel_size = 0.001
    pc_o3d_downsampled = pc_o3d.voxel_down_sample(voxel_size)

    # Open3DのPointCloudからPointCloud2に変換
    header = Header()
    header.frame_id = data.header.frame_id
    downsampled_points = pc_o3d_downsampled.points
    downsampled_pc2 = point_cloud2.create_cloud_xyz32(header, downsampled_points)

    # ダウンサンプリングした点群をPublish
    downsampled_pub.publish(downsampled_pc2)

if __name__ == '__main__':
    rospy.init_node('downsample_node', anonymous=True)
    threshold = 1

    # TransformListenerを初期化
    listener = TransformListener()

    # Subscriberの設定
    pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, downsample_callback)

    # Publisherの設定
    downsampled_pub = rospy.Publisher('downsampled_pc', PointCloud2, queue_size=1)

    rospy.spin()