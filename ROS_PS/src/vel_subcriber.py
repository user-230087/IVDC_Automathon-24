#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist

class sub_node :

    def __init__(self):
        self.vel_sub =  rospy.Subscriber("smb_velocity_controller/cmd_vel",Twist,self.callback)
        
    def callback(self, Twist):
        rospy.loginfo(rospy.get_caller_id() + "Velocity: %s",Twist)
        print('Callback executed!')

def main():
    vsub = sub_node()
    rospy.init_node('listener', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 



