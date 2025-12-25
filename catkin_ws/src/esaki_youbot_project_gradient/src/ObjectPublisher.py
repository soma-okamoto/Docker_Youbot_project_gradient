#!/usr/bin/env python3
# coding: UTF-8
from cProfile import label
import rospy
from std_msgs.msg import Float32MultiArray


def ObjectDetection():
    label = "petbottle"
    x = 0
    y = 0
    z = 1
    w = 0.3
    h = 0.4

    return label,x,y,z,w,h


def Objectinfo(label,x,y,z,w,h):

    objectinfo = []
    label = "book"
    x = x
    y = y
    z = z
    w = w
    h = h

    labelindex = {'petbottle' : 0, 'book' : 1}

    objectinfo.append(labelindex[label])    
    objectinfo.append(x)
    objectinfo.append(y)
    objectinfo.append(z)
    objectinfo.append(w)
    objectinfo.append(h)

    return objectinfo


        
# メイン
def main():
    # パブリッシャーの生成
    pub = rospy.Publisher('/multi_command', Float32MultiArray, queue_size=10)

    # ノードの初期化
    rospy.init_node("ObjectDetection", anonymous=True)

    array=[]


    label,x,y,z,w,h = ObjectDetection()

    objectinfo = Objectinfo(label,x,y,z,w,h)
    

    array_forPublish = Float32MultiArray(data=objectinfo)
    pub.publish(array_forPublish)
        

if __name__ == "__main__":
    main()

    import rospy

