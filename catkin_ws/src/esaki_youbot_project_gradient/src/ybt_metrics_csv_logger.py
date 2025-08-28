#!/usr/bin/env python3
import rospy, csv, os, time
from std_msgs.msg import Float32MultiArray, String

header = None
writer = None
f = None

def names_cb(msg):
    global header, writer, f
    if header is None:
        header = ['rostime'] + [s.strip() for s in msg.data.split(',')]
        ts = time.strftime("%Y%m%d_%H%M%S")
        out = f"/tmp/ybt_metrics_{ts}.csv"
        f = open(out, 'w', newline='')
        writer = csv.writer(f)
        writer.writerow(header)
        rospy.loginfo(f"[ybt_metrics_csv_logger] writing to {out}")

def metrics_cb(msg):
    global writer, f
    if writer is None: return
    row = [rospy.get_time()] + list(msg.data)
    writer.writerow(row)
    f.flush()

if __name__ == "__main__":
    rospy.init_node("ybt_metrics_csv_logger")
    rospy.Subscriber("/ybt_metric_names", String, names_cb, queue_size=1)
    rospy.Subscriber("/ybt_metrics", Float32MultiArray, metrics_cb, queue_size=50)
    rospy.spin()
    if f: f.close()
