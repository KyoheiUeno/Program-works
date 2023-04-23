#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import angles
import math
import sys
import roslib
import rospy
import nav_msgs.msg
import sensor_msgs.msg
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_wait=1.0):
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = msg_type

    def get_msg(self):
        message = None
        try:
            message = rospy.wait_for_message(
                self.topic, self.msg_type, self.msg_wait)
        except rospy.exceptions.ROSException as e:
            rospy.logdebug(e)
        return message

def get_yaw(odom):
    q = (odom.pose.pose.orientation.x,
         odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z,
         odom.pose.pose.orientation.w)
    euler = euler_from_quaternion(q)
    return euler[2]

def go_straight_by_distance(distance, time_limit=999, linear_vel=0.4, topic='/odom', cmd_vel="/cmd_vel", msg_wait=1.0):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    sensor_msg = SensorMessageGetter(
        topic, nav_msgs.msg.Odometry, msg_wait)
    pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    vel = Twist()
    vel.linear.x = linear_vel
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0
    diff = 0
    msg = sensor_msg.get_msg()
    goalx = msg.pose.pose.position.x
    goaly = msg.pose.pose.position.y
    total = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        msg = sensor_msg.get_msg()
        if msg is not None:
            if total > distance:
                break
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = get_yaw(msg)
            total = math.hypot(x - goalx,y - goaly)
            rospy.loginfo(
                'Recv odometry. (x, y, theta) d = (%.2f, %.2f, %.2f)%.2f', x, y, math.degrees(yaw), total)
            # http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html をよく見よう。
            # msg.pose.pose.position.x などで座標が分かる。
            
        pub.publish(vel)
        rospy.sleep(0.1)
        diff = rospy.get_time() - start_time
    vel.linear.x = 0.0
    pub.publish(vel)

def turn_by_angle(angle, time_limit=999, angular_vel=math.radians(30), topic='/odom', cmd_vel="/cmd_vel", msg_wait=1.0): # 初期値はどうする？
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    sensor_msg = SensorMessageGetter(
        topic, nav_msgs.msg.Odometry, msg_wait)
    pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = angular_vel
    diff = 0
    msg = sensor_msg.get_msg()
    goalx = msg.pose.pose.position.x
    goaly = msg.pose.pose.position.y
    yaw_pre = get_yaw(msg)
    total = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        msg = sensor_msg.get_msg()
        if msg is not None:
            if total > angle:
                break
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = get_yaw(msg)
            d = abs(angles.normalize_angle(yaw - yaw_pre))
            # d に１フレームで回転した角度の絶対値が入っている。これを累積する。
            yaw_pre = yaw
            total = total + d
            rospy.loginfo(
                'Recv odometry. (x, y, theta) d = (%.2f, %.2f, %.2f)%.2f', x, y, math.degrees(yaw), total)
            # http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html をよく見よう。
            # msg.pose.pose.position.x などで座標が分かる。
            
        pub.publish(vel)
        rospy.sleep(0.1)
        diff = rospy.get_time() - start_time
    vel.linear.x = 0.0
    pub.publish(vel)

def check_command(time_limit, topic='/chatter', msg_wait=1.0):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    sensor_msg = SensorMessageGetter(
        topic, String, msg_wait)
    diff = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        msg = sensor_msg.get_msg()
        if msg is not None:
            rospy.loginfo("Recv command %s", msg.data)
            if msg.data == "go_forward_1.0":
                go_straight_by_distance(1.0, 2.5)

            if msg.data == "go_back_1.0":
                go_straight_by_distance(1.0, 2.5, -0.4)

            if msg.data == "turn_left_90":
                turn_by_angle(math.radians(90), angular_vel=math.radians(-30))

            if msg.data == "turn_right_90":
                turn_by_angle(math.radians(-90), angular_vel=math.radians(-30))

            if msg.data == "p1":
                ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                coord_type = "map" # マップ座標系
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = coord_type
                goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
                goal.target_pose.pose.position.x = 7
                goal.target_pose.pose.position.y = 4
                q = quaternion_from_euler(0, 0, math.radians(60))
                goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                rospy.loginfo("Sending goal")
                ac.send_goal(goal)
                finished = ac.wait_for_result(rospy.Duration(30))
                state = ac.get_state()
            
            if msg.data == "p2":
                ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                coord_type = "map" # マップ座標系
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = coord_type
                goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
                goal.target_pose.pose.position.x = 4
                goal.target_pose.pose.position.y = 4
                q = quaternion_from_euler(0, 0, math.radians(60))
                goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

            if msg.data == "p3":
                ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                coord_type = "map" # マップ座標系
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = coord_type
                goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
                goal.target_pose.pose.position.x = 3.5
                goal.target_pose.pose.position.y = 2
                q = quaternion_from_euler(0, 0, math.radians(60))
                goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

            return msg.data   
        rospy.sleep(0.1)

        
        diff = rospy.get_time() - start_time
    return None


def main():
    rospy.init_node('recv_commands')
    rospy.loginfo("C19XXX 工大 太郎")
    # Action Client
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒）
    while not ac.wait_for_server(rospy.Duration(5)):
        rospy.loginfo("Waiting for the move_base action server to come up")

    rospy.loginfo("The server comes up")

    while not rospy.is_shutdown():
        command = check_command(30)
        if command is None:
            continue


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
