#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import numpy as np
import struct

def float_to_bytearray(float_list):
    byte_list = []
    for f in float_list:
        b = struct.pack('f', f)
        byte_list.extend(b)
    return byte_list

def downsample_callback(data):
    # PointCloud2からOpen3DのPointCloudに変換
    # gen1 = point_cloud2.read_points(data, skip_nans=True)
    # gen2 = point_cloud2.read_points(data, skip_nans=True)
    pointcloud = point_cloud2.read_points(data,skip_nans=True)
    # pcl1 = np.array(list(gen1))
    # pcl2 = np.array(list(gen2)) 

    # pc = np.concatenate([pcl1,pcl2],axis=0)
    
    points = []
    colors = []

    #ARマーカーからの距離にする
    # for p in pc:
    for p in pointcloud:
        point = np.array([p[0],p[1],p[2]])
        camera_pos = np.array([0,0,0])
        
        if(np.linalg.norm(point - camera_pos > threshold)):
            continue
        # rgbフィールドがある場合はカラー情報を取得する
        if len(p) >= 4 and data.fields[3].name == 'rgb':
            rgb = struct.unpack('I', struct.pack('f', p[3]))[0]
            # RGB値を分解する
            red = (rgb & 0x00FF0000) >> 16
            green = (rgb & 0x0000FF00) >> 8
            blue = rgb & 0x000000FF
            colors.append([red, green, blue])
            points.append(point)
        else:
            colors.append([0, 0, 0])
            points.append(point)

    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(points)
    pc_o3d.colors = o3d.utility.Vector3dVector(colors)

    # VoxelDownSampleでダウンサンプリング
    voxel_size = 0.001
    pc_o3d_downsampled = pc_o3d.voxel_down_sample(voxel_size)

    # Open3DのPointCloudからPointCloud2に変換
    header = Header()
    header.frame_id = data.header.frame_id
    downsampled_points = pc_o3d_downsampled.points
    downsampled_colors = pc_o3d_downsampled.colors
    # downsampled_pc2 = point_cloud2.create_cloud_xyzrgb(header, downsampled_points, downsampled_colors)

    # PointCloud2メッセージの作成
    header = Header()
    header.frame_id = "camera_frame"  # フレームIDを設定する
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 16, PointField.UINT32, 1)]  # RGB値をfloat型で指定する
    downsampled_pc2 = PointCloud2(header=header, fields=fields, height=1, width=len(downsampled_points), is_dense=False)

    # データをセットする
    points_list = []
    for i in range(len(downsampled_points)):
        point = [downsampled_points[i][0], downsampled_points[i][1], downsampled_points[i][2], downsampled_colors[i][0], downsampled_colors[i][1], downsampled_colors[i][2]]
        # RGB値をuint8に変換する
        rbt_list = [ downsampled_colors[i][0], downsampled_colors[i][1], downsampled_colors[i][2],255]
        rgb = struct.pack('BBBB', *map(int, rbt_list[0:4]))
        # RGB値をuint32に変換する
        rgb = struct.unpack('I', rgb)[0]
        point[3] = rgb
        points_list.extend(point)
    points_bytearray = float_to_bytearray(points_list)
    downsampled_pc2.data = bytearray(points_bytearray)
    downsampled_pc2.point_step = 24  # 1点あたりのバイト数を設定する
    downsampled_pc2.row_step = downsampled_pc2.point_step * downsampled_pc2.width

    # downsampled_pc2 = point_cloud2.create_cloud_xyz32(header, downsampled_points)

    # ダウンサンプリングした点群をPublish
    downsampled_pub.publish(downsampled_pc2)

if __name__ == '__main__':
    rospy.init_node('downsample_node', anonymous=True)
    threshold = 1
    
    # Subscriberの設定
    pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, downsample_callback)

    # Publisherの設定
    downsampled_pub = rospy.Publisher('downsampled_pc', PointCloud2, queue_size=1)

    rospy.spin()