#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs import Point, Twist
import math

x=0.0
y=0.0
theta = 0.0
def newOdom (msg):
    global x
    global y 
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    rot_q = msg.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])




rospy.init__node("spped controller")


sub = rospy.Subscriber("/odometry/filterred", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
speed = Twist()

r = rospy.Rate(4)

goal = Point ()
goal.x = 20
goal.y = 0

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = math.atan2 (inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()

