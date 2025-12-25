#!/usr/bin/env python3
import pyrealsense2 as rs
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from brics_actuator.msg import JointPositions

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import tf

color_intr = rs.intrinsics()

def DegToRad(th):
    rad=(np.pi/180)*th
    return rad

def RadToDeg(rad):
    deg = rad *(180/np.pi)
    return deg

# 回転行列
def rot_x(th):
  
  c = np.cos(th)
  s = np.sin(th)
  
  Rx = np.array([[1, 0, 0],
               [0, c, -s],
               [0, s, c]])
  
  return Rx

def rot_y(th):
  
  c = np.cos(th)
  s = np.sin(th)
  
  Ry = np.array([[c, 0, s],
               [0, 1, 0],
               [-s, 0, c]])

  return Ry

def rot_z(th):
  
  c = np.cos(th)
  s = np.sin(th)
  
  Rz = np.array([[c, -s, 0],
               [s, c, 0],
               [0, 0, 1]])  
  
  return Rz

def CoordinateTransformation(x,y,z,color_intr):
    print(x,y,z)
    world_point = rs.rs2_deproject_pixel_to_point(color_intr , [x,y],z)
    from_eef_to_object=np.array([[-world_point[1]],[world_point[0]],[z]])
    q5= 0
    q4= -90
    q3= -70
    q2= 40
    q1= -60

    test1 = -(cameraPosition.positions[0].value -DegToRad(169))
    test2 = cameraPosition.positions[1].value - DegToRad(65)
    test3 = cameraPosition.positions[2].value + DegToRad(146)
    test4 = cameraPosition.positions[3].value - DegToRad(102.5)
    test5 = -(cameraPosition.positions[4].value - DegToRad(167.5))
    
    rot_camera=rot_y(DegToRad(-8))
    trans5=np.array([[0.06],[0.033],[0.08]])
    rot_5=rot_z(test5)

    trans4=np.array([[0],[0],[0.13]])
    rot_4=rot_y(test4)

    trans3=np.array([[0],[0],[0.135]])
    rot_3=rot_y(test3)

    trans2=np.array([[0],[0],[0.155]])
    rot_2=rot_y(test2)

    trans1=np.array([[0.033],[0],[0.019]])
    rot_1=rot_z(test1)
    #0.091は地面からbase_linkへの並進移動
    trans0=np.array([[-0.147],[0],[0.152+0.091]])
    rot_0=rot_z(DegToRad(180))

    #baseから見たobject
    object2base=trans0+rot_0@(rot_1@(trans1+rot_2@(trans2+rot_3@(trans3+rot_4@(trans4+rot_5@(rot_camera@(trans5+from_eef_to_object)))))))

    return object2base

def AR_position_callback(msg):
    global MarkerPosition
    #rospy.loginfo("Message '{}' recieved".format(msg))
    MarkerPosition = msg

def position_callback(msg):
    global cameraPosition
    #rospy.loginfo("Message '{}' recieved".format(msg))
    cameraPosition = msg

def camerainfo_callback(msg_camerainfo):
        
        global color_intr
        color_intr.height =  msg_camerainfo.height
        color_intr.width = msg_camerainfo.width
        color_intr.fx = msg_camerainfo.K[0]
        color_intr.fy = msg_camerainfo.K[4]
        color_intr.ppx = msg_camerainfo.K[2]
        color_intr.ppy = msg_camerainfo.K[5]
        color_intr.model = rs.distortion.inverse_brown_conrady
        color_intr.coeffs = msg_camerainfo.D


def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """

    e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    euler = [e[0],e[1],e[2]]

    return euler

if __name__ == "__main__":
    rospy.init_node("get_ARmarker_position", anonymous=True)
    rate = rospy.Rate(10)
    #MarkerPosition_sub = rospy.Subscriber("/visualization_marker", Marker, AR_position_callback)
    #MarkerPosition = Marker()
    #cameraPosition_sub = rospy.Subscriber("arm_2/arm_controller/position_command", JointPositions, position_callback)
    #cameraPosition = JointPositions()
    #cameraInfo_sub = rospy.Subscriber("/camera/color/camera_info",CameraInfo,camerainfo_callback)
    #camerainfo = CameraInfo()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    while not rospy.is_shutdown():
        rospy.sleep(0.2)
        #pos_x,pos_y,pos_z = MarkerPosition.pose.position.x,MarkerPosition.pose.position.y,MarkerPosition.pose.position.z
        #ori_x,ori_y,ori_z,ori_w = MarkerPosition.pose.orientation.x,MarkerPosition.pose.orientation.y,MarkerPosition.pose.orientation.z,MarkerPosition.pose.orientation.w

        armarcker = tf2_geometry_msgs.PoseStamped()
        armarcker.header.frame_id = "ar_marker_0"
        
        armarcker.header.stamp = rospy.Time.now()
        armarckerPosition = rospy.Publisher("ARmarker_position",PoseStamped,queue_size = 5)
        
        armarcker_pose = PoseStamped()
        
        try:
            global_pose = tfBuffer.lookup_transform("map","ar_marker_0",rospy.Time(0))

            quaternion = [global_pose.transform.rotation.x,global_pose.transform.rotation.y,global_pose.transform.rotation.z,global_pose.transform.rotation.w]
            euler = quaternion_to_euler(quaternion)

            #print(euler)
            print(global_pose)
            armarcker_pose.pose.position.x,armarcker_pose.pose.position.y,armarcker_pose.pose.position.z = global_pose.transform.translation.x,global_pose.transform.translation.y,global_pose.transform.translation.z
            armarcker_pose.pose.orientation.x,armarcker_pose.pose.orientation.y,armarcker_pose.pose.orientation.z,armarcker_pose.pose.orientation.w = global_pose.transform.rotation.x,global_pose.transform.rotation.y,global_pose.transform.rotation.z,global_pose.transform.rotation.w
            
            armarckerPosition.publish(armarcker_pose)
            #object2base = CoordinateTransformation(global_pose.pose.position.x,global_pose.pose.position.y,global_pose.pose.position.z,color_intr)
            #print(pos_x,pos_y,pos_z)
            #print(object2base)
        except(tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rospy.logwarn("tf not found")
        rate.sleep()
            
        