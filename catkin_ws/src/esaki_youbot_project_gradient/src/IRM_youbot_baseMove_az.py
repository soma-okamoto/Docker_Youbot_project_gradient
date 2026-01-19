#!/usr/bin/env python3
import rospy
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg       import Path
from geometry_msgs.msg  import Twist, PointStamped
from std_msgs.msg       import Bool
from tf.transformations import euler_from_quaternion

class IRMEditRouteExecutor:
    def __init__(self):
        rospy.init_node("irm_edit_route_executor", anonymous=False)

        # パラメータ
        self.route_topic = rospy.get_param("~route_topic", "/IRM_Edit_Route")
        self.cmd_topic   = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.frame_map   = rospy.get_param("~map_frame", "map")
        self.frame_base  = rospy.get_param("~base_frame", "base_link")
        self.waypoint_tolerance = rospy.get_param("~tolerance", 0.1)  # [m]
        self.max_lin_vel       = rospy.get_param("~max_lin_vel", 0.3) # [m/s]
        self.max_ang_vel       = rospy.get_param("~max_ang_vel", 0.5) # [rad/s]

        # Publishers & Subscribers
        self.cmd_pub  = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.done_pub = rospy.Publisher("route_done", Bool, queue_size=1)
        self.sub      = rospy.Subscriber(self.route_topic, Path, self.route_callback, queue_size=1)

        # TF
        self.tf_buf  = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf)

        rospy.loginfo(f"[IRMEditRouteExecutor] Listening on {self.route_topic}, publishing {self.cmd_topic}")
        rospy.spin()

    def route_callback(self, msg: Path):
        rospy.loginfo(f"[IRMEditRouteExecutor] Received edited route: {len(msg.poses)} points")
        # Path の各 PoseStamped をワールド座標でリスト化
        waypoints = [
            (ps.pose.position.x, ps.pose.position.y, ps.pose.orientation)
            for ps in msg.poses
        ]
        # 移動実行
        self.follow_waypoints(waypoints)
        # 完了を通知
        self.done_pub.publish(Bool(data=True))

    def follow_waypoints(self, waypoints):
        rate = rospy.Rate(10)
        for idx, (wx, wy, orientation_q) in enumerate(waypoints):
            rospy.loginfo(f"[IRMEditRouteExecutor] Heading to waypoint {idx}: ({wx:.2f}, {wy:.2f})")
            # 各ウェイポイントへ到達するまでループ
            while not rospy.is_shutdown():
                # 1) 現在のロボット位置・方位を取得
                trans = self.lookup_transform(self.frame_map, self.frame_base)
                rx = trans.transform.translation.x
                ry = trans.transform.translation.y
                q  = trans.transform.rotation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                # 2) 目標方向を計算
                dx = wx - rx
                dy = wy - ry
                dist = math.hypot(dx, dy)
                if dist < self.waypoint_tolerance:
                    break


                vel = Twist()
                # グローバル方向ベクトルを正規化して max_lin_vel を乗算
                vx_map = dx / dist * self.max_lin_vel
                vy_map = dy / dist * self.max_lin_vel
                # ロボット前方(x)・左方向(y)へ変換（yaw は先ほど取得した yaw）
                vel.linear.x =  math.cos(yaw) * vx_map + math.sin(yaw) * vy_map
                vel.linear.y = -math.sin(yaw) * vx_map + math.cos(yaw) * vy_map
                # yaw 制御はしない
                vel.angular.z = 0.0
                self.cmd_pub.publish(vel)

                rate.sleep()

            # 停止コマンド
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.2)

            # 各ウェイポイントごとに向きを合わせる
            goal_q = orientation_q
            # euler 取得
            _, _, goal_yaw = euler_from_quaternion([
                goal_q.x, goal_q.y, goal_q.z, goal_q.w
            ])
            self.rotate_to(goal_yaw)


    def rotate_to(self, target_yaw):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            trans = self.lookup_transform(self.frame_map, self.frame_base)
            q  = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            err = math.atan2(math.sin(target_yaw - yaw),
                             math.cos(target_yaw - yaw))
            if abs(err) < 0.05:
                break
            twist = Twist()
            twist.angular.z = max(-self.max_ang_vel,
                                   min(self.max_ang_vel, err))
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def lookup_transform(self, from_frame, to_frame):
        try:
            return self.tf_buf.lookup_transform(
                from_frame, to_frame, rospy.Time(0), rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return None

if __name__ == "__main__":
    try:
        IRMEditRouteExecutor()
    except rospy.ROSInterruptException:
        pass
