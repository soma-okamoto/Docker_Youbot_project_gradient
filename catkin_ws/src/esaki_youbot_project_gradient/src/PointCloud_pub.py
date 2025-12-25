#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import numpy as np

def downsample_callback(data):
    # PointCloud2からOpen3DのPointCloudに変換
    pc = point_cloud2.read_points(data, skip_nans=True)
    points = []
    
    camera_pos = np.array([0,0,0])
    for p in pc:
        point = np.array([p[0],p[1],p[2]])
        
        

        if(np.linalg.norm(point - camera_pos > threshold)):
            continue
        points.append(point)

    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(points)

    # VoxelDownSampleでダウンサンプリング
    voxel_size = 0.001
    pc_o3d_downsampled = pc_o3d.voxel_down_sample(voxel_size)

    # Open3DのPointCloudからPointCloud2に変換
    header = Header()
    header.frame_id = data.header.frame_id
    downsampled_points = pc_o3d.points
    downsampled_pc2 = point_cloud2.create_cloud_xyz32(header, downsampled_points)

    # ダウンサンプリングした点群をPublish
    downsampled_pub.publish(downsampled_pc2)

    # # ダウンサンプリング後の点群データを保存
    # output_path = './pointcloud4.ply'
    # o3d.io.write_point_cloud(output_path, pc_o3d_downsampled)

if __name__ == '__main__':
    rospy.init_node('downsample_node', anonymous=True)
    threshold = 1
    
    # Subscriberの設定
    pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, downsample_callback)

    # Publisherの設定
    downsampled_pub = rospy.Publisher('downsampled_pc', PointCloud2, queue_size=1)

    rospy.spin()