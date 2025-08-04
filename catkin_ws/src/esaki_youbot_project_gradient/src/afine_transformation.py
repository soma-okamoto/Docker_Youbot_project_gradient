#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray



def before_callback(before_msg):
    global before_pose
    #rospy.loginfo("Message '{}' recieved".format(before_msg))
    before_pose = before_msg

def after_callback(after_msg):
    global after_pose
    #rospy.loginfo("Message '{}' recieved".format(after_msg))
    after_pose = after_msg

def object_callback(obj_msg):
    global obj_pose
    # rospy.loginfo("Message '{}' recieved".format(obj_msg))
    obj_pose = obj_msg

def _multiarray2numpy(pytype, dtype, multiarray):
    """Convert multiarray to numpy.ndarray"""
    dims = map(lambda x: x.size, multiarray.layout.dim)
    return np.array(multiarray.data, dtype=pytype).reshape(dims).astype(dtype)

def Afine_Transformation(before,after,obj):

    x_after = []
    y_after = []
    z_after = []

    x_before = []
    y_before = []
    z_before = []
    label_before = []
    id_before = []

    for i in range(int(len(before.data)/3)):
        x_before.append(before.data[i * 3] - before.data[0])
        y_before.append(before.data[i * 3 +1] - before.data[1])
        z_before.append(before.data[i * 3 + 2] - before.data[2])

    for j in range(int(len(after.data)/3)):
        x_after.append(after.data[j*3] - after.data[0])
        y_after.append(after.data[j*3 +1] - after.data[1])
        z_after.append(after.data[j*3 +2] - after.data[2])

    dst = np.array([x_after[0], y_after[0],z_after[0], x_after[1], y_after[1], z_after[1],x_after[2], y_after[2],z_after[2],x_after[3],y_after[3],z_after[3]]).T

    before_list = []
    for i in range(4):

        x = [x_before[i],y_before[i],z_before[i],1,0,0,0,0,0,0,0,0]
        y = [0,0,0,0,x_before[i],y_before[i],z_before[i],1,0,0,0,0]
        z = [0,0,0,0,0,0,0,0,x_before[i],y_before[i],z_before[i],1]
        before_list.append(x)
        before_list.append(y)
        before_list.append(z)

    mat = np.array(before_list)

    ans = np.matmul(np.linalg.pinv(mat), dst)

    affine = np.array([[ans[0], ans[1], ans[2],ans[3]],
                    [ans[4], ans[5], ans[6],ans[7]],
                    [ans[8],ans[9],ans[10],ans[11]],
                    [0, 0, 0, 1]])


    after_object_position = []
    for k in range(int(len(obj.data)/3)):
        obj_x = obj.data[k*3] - before.data[0]
        obj_y = obj.data[k*3 +1] - before.data[1]
        obj_z = obj.data[k*3 +2] - before.data[2]

        plot = np.array([obj_x,obj_y,obj_z,1])

        #print(plot)

        afine_after = np.dot(affine,plot)
        afine_after.tolist()
        after_object_position.append(afine_after[0] + after.data[0])
        after_object_position.append(afine_after[1] + after.data[1])
        after_object_position.append(afine_after[2] + after.data[2])

    return after_object_position


if __name__ == '__main__':
    rospy.init_node("afine_transform", anonymous=True)
    pub = rospy.Publisher('/after_calibration_command', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10)
    after_pose_sub = rospy.Subscriber("/after_command",Float32MultiArray,after_callback)
    before_pose_sub = rospy.Subscriber("/before_command",Float32MultiArray,before_callback)
    obj_pose_sub = rospy.Subscriber("/calibration_command",Float32MultiArray,object_callback)

    before_pose = Float32MultiArray()
    after_pose = Float32MultiArray()
    obj_pose = Float32MultiArray()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        if(len(before_pose.data) > 2):
            afine_after = Afine_Transformation(before_pose,after_pose,obj_pose)

            array_forPublish = Float32MultiArray(data=afine_after)
            pub.publish(array_forPublish)
            print(afine_after)