#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

import csv

def before_callback(before_msg):
    global before_pose_list
    #rospy.loginfo("Message '{}' recieved".format(before_msg))
    before_pose_list = before_msg

def after_callback(after_msg):
    global after_pose_list
    #rospy.loginfo("Message '{}' recieved".format(after_msg))
    after_pose_list = after_msg

def object_callback(obj_msg):
    global obj_pose_list
    #rospy.loginfo("Message '{}' recieved".format(obj_msg))
    obj_pose_list = obj_msg

def save_csv(before,after,obj):

    before_list = []
    after_list = []
    obj_list = []

    for i in range(int(len(before.data)/3)):
        before_list.append(before.data[i * 3] - before.data[0])
        before_list.append(before.data[i * 3 +1] - before.data[1])
        before_list.append(before.data[i * 3 + 2] - before.data[2])

    for j in range(int(len(after.data)/3)):
        # after_list.append(after.data[j*3] - before.data[0])
        # after_list.append(after.data[j*3 +1] - before.data[1])
        # after_list.append(after.data[j*3 +2] - before.data[2])
        after_list.append(after.data[j*3])
        after_list.append(after.data[j*3 +1])
        after_list.append(after.data[j*3 +2])

    for k in range(int(len(obj.data)/3)):
        # obj_x = obj.data[k*3] - before.data[0]
        # obj_y = obj.data[k*3 +1]- before.data[1]
        # obj_z = obj.data[k*3 +2]- before.data[2]
        obj_x = obj.data[k*3]
        obj_y = obj.data[k*3 +1]
        obj_z = obj.data[k*3 +2]
        obj_list.append(obj_x)
        obj_list.append(obj_y)
        obj_list.append(obj_z)

    # 行ごとに配列を結合する
    rows = zip(after_list, obj_list)

    # CSVファイルを作成する
    with open('output_3d.csv', 'w', newline='') as csvfile:
        # CSVファイルに書き込むためのwriterオブジェクトを作成する
        writer = csv.writer(csvfile)
        # 各行を書き込む
        for row in rows:
            writer.writerow(row)

if __name__ == '__main__':
    saved = False
    rospy.init_node("save_csv_node", anonymous=True)

    rate = rospy.Rate(10)
    before_pose_sub = rospy.Subscriber("/before_command",Float32MultiArray,before_callback)
    after_pose_sub = rospy.Subscriber("/after_calibration_command",Float32MultiArray,after_callback)
    obj_pose_sub = rospy.Subscriber("/calibration_command",Float32MultiArray,object_callback)

    before_pose_list = Float32MultiArray()
    after_pose_list = Float32MultiArray()
    obj_pose_list = Float32MultiArray()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

        if(saved == False):
            save_csv(before_pose_list,after_pose_list,obj_pose_list)
            print("save_csv")
            saved = True
        
