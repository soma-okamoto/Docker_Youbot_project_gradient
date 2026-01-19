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
from std_msgs.msg import Header,Bool
from sensor_msgs import point_cloud2
import numpy as np
from tf import TransformListener
from geometry_msgs.msg import PointStamped,PoseStamped, Quaternion
import signal
import sys
import tf.transformations as tf_trans

import asyncio

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
    

# 非同期処理を行う関数は、async と付けなければならない
async def merge_pointcloud(pointcloud_data):
    global target,source,union
    
    merge_status_pub.publish(True)
    accumulated_pc = point_cloud2.read_points(pointcloud_data,skip_nans=True)
    if(target == None):
        print("target save")
        
        points = []

        camera_pos = np.array([0,0,0])

        current_pose = get_current_pose()

        base_trans,rot = current_pose.pose.position,current_pose.pose.orientation

        # 四元数の作成
        quaternion = [rot.x, rot.y, rot.z, rot.w]

        # 回転行列の作成
        rotation_matrix = tf_trans.quaternion_matrix(quaternion)[:3, :3]


        for p in accumulated_pc:
            point = np.array([p[0], p[1], p[2]])

            # カメラ位置からの距離がしきい値以上の点のみを抽出する
            threshold = 2
            if np.linalg.norm(point - camera_pos) > threshold:
                continue
            
            # # ベクトルの回転
            transformed_point = np.dot(rotation_matrix, point) + np.array([base_trans.x, base_trans.y, base_trans.z])

            points.append(transformed_point)

        # Open3DのPointCloudに変換
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(points)
        target.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1,
            max_nn=30
        )
        )

        union = target
    else:
        print("source Save")

        points = []

        camera_pos = np.array([0,0,0])

        for p in accumulated_pc:
            point = np.array([p[0], p[1], p[2]])

            # カメラ位置からの距離がしきい値以上の点のみを抽出する
            threshold = 2
            if np.linalg.norm(point - camera_pos) > threshold:
                continue
            points.append(point)

        # Open3DのPointCloudに変換
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(points)
        source.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1,
            max_nn=30
        )
        )


    if(source != None):
        print("Merge PointCloud")
        voxel_size = 0.05  # means 5cm for this dataset
        source, union, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
            voxel_size,source,union)

        result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

        result_icp = refine_registration(source, union, source_fpfh, target_fpfh,
                                        voxel_size,result_ransac)

        source.transform(result_icp.transformation)
            
        preUnion = union

        #not_overlap_pcd1 = find_not_overlap(union, source)

        union += source

        #target = remove_overlapping_points_using_kdtree(union,preUnion)
        target = source
        #target += source
        print("Done Merge")
    
        # # # VoxelDownSampleでダウンサンプリング
    voxel_size = 0.02

    pc_o3d_downsampled = target.voxel_down_sample(voxel_size)

    downsampled_points = pc_o3d_downsampled.points
    downsampled_pc2 = point_cloud2.create_cloud_xyz32(header, downsampled_points)

    #     # ダウンサンプリングした点群をPublish
    downsampled_pub.publish(downsampled_pc2)
    print("downsample publish : " + str(len(target.points)))
    merge_status_pub.publish(False)


def downsample_callback(data):
    
    global pointcloud_data
    pointcloud_data =data


    # Open3DのPointCloudからPointCloud2に変換
    global header
    header = Header()
    header.frame_id = data.header.frame_id

    pc_sub.unregister()

async def PointCloud_Task(pointcloud_data):
    
    await asyncio.gather( # 処理が全部終わるまで待つ
        merge_pointcloud(pointcloud_data)
    )

def find_not_overlap(pcd1, pcd2, threshold=0.01):
    not_overlap_pcd = o3d.geometry.PointCloud()
    kd_tree = o3d.geometry.KDTreeFlann(pcd2)

    for point in pcd1.points:
        [k, idx, _] = kd_tree.search_radius_vector_3d(point, threshold)
        if k <= 0:
            not_overlap_pcd.points.append(point)
    print(len(not_overlap_pcd.points))

    return not_overlap_pcd

def merge_pointcloud_callback(msg):
    global merge_pointcloud_status
    #rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    merge_pointcloud_status = msg

# 重複する点を削除する
def remove_overlapping_points_using_kdtree(source_point, union, threshold=1e-5):
    """
    Remove points in source that are also in union using a KDTree for efficient nearest neighbor search.
    threshold: Distance threshold to consider points as overlapping.
    """
    print("remove start")
    # Build KDTree for union point cloud
    union_tree = o3d.geometry.KDTreeFlann(union)

    # Prepare a mask to mark points to keep
    keep_mask = np.ones(len(source_point.points), dtype=bool)

    for i, point in enumerate(source_point.points):
        # Find nearest neighbor in union
        [k, idx, _] = union_tree.search_radius_vector_3d(point, threshold)
        
        # If there are points within the threshold radius, mark for removal
        if k > 0:
            keep_mask[i] = False

    # Filter out points marked for removal
    source_point.points = o3d.utility.Vector3dVector(np.asarray(source_point.points)[keep_mask])
    print("remove end")
    return source_point

if __name__ == '__main__':

    global target,source,union
    x = 0
    y = 0
    # theta = math.pi/6
    theta = 0

    # ノードを初期化する
    rospy.init_node('move_base_test', anonymous=True)

    target = None
    source = None
    union = None
    pointcloud_status = True
    merge_status_pub = rospy.Publisher('merge_pointcloud_status',Bool,queue_size=1)
    merge_status_sub = rospy.Subscriber('merge_pointcloud_status',Bool,merge_pointcloud_callback)

    merge_status_pub.publish(False)

    goal_status = False

    downsampled_pub = rospy.Publisher('downsampled_pc', PointCloud2, queue_size=1)
    

    goal_pub = rospy.Publisher('goal_Pub',Bool,queue_size=1)



    merge_pointcloud_status = Bool()

    while not rospy.is_shutdown():

        # # ノードを初期化する
        # rospy.init_node('move_base_test', anonymous=True)
        # Publisherの設定

        # 初期位置から半径1mの円周上の目標位置を設定する
        num_points = 12
        radius = 0.5
        merge_status_pub.publish(merge_pointcloud_status)
            # Subscriberの設定

        pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, downsample_callback)
        pointcloud_data = PointCloud2()

        # TransformListenerを初期化
        listener = TransformListener()
        if(merge_pointcloud_status.data == False):   
            if(pointcloud_status == True):
                if( goal_status == False):
                    if(x <= 1.0):
                        goal_msg = Bool()


                        result = move_base_client(x,y, theta)

                        if result:
                            rospy.sleep(1)
                            goal_msg.data = True
                            goal_pub.publish(goal_msg)
                            goal_msg.data = False

                            print(len(pointcloud_data.data))
                            asyncio.run(PointCloud_Task(pointcloud_data))

                            print("registration")
                            rospy.loginfo("Goal execution Done")
                            rospy.loginfo("x  : " + str(x))

                            x += 0.4
                            y = 0.0
                            theta =0
                            
                            result = None
                            rospy.sleep(2)
                            goal_pub.publish(goal_msg)
                    else:

                        # Subscriberの設定
                        #pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, downsample_callback)
                        
                        x = 0.0
                        y = 0.0

                        print("registration")
                        rospy.loginfo("Goal execution Done")
                        rospy.loginfo("x  : " + str(x))
                        rospy.sleep(10)
                        goal_status = True
                        #break
                else:
                    result = move_base_client(x,y, theta)

                    if result:
                        break
                

            # break

        # if (x >=1.0 and x <=1.2):
        #     y = 0
        #     theta = - math.pi/6
        
        # if (x >=1.2 and x <=1.4):
        #     y = -0.2
        #     theta = - math.pi/3

        # if(x >=1.4 and x <= 1.6):
        #     y = -0.6
        #     theta = - math.pi/2

        #if (x >=1.0):

            # 類似点をマージする閾値
            #threshold = 1

            # ポイントクラウドを合わせる
            #aligned_pointcloud = align_pointclouds(accumulated_pc)

            #merged_points = merge_similar_points(accumulated_pc, aligned_pointcloud)
            #print("saving")
            #output_path = './Save_move_pointcloud-global-registration7.ply'
            #o3d.io.write_point_cloud(output_path, target)
            #break

    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation test finished.")
    # rospy.spin()