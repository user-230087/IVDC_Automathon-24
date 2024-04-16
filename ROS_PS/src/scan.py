#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan


class sc_node :

    def __init__(self):
        self.vel_sub =  rospy.Subscriber('scan', LaserScan)
        
    def callback(msg):
        deg=[0,90,180]
        vl=[0,360,719]
        length=len(deg)
        for i in range(length):
            print('Value at ', deg[i], 'is: ', msg.ranges[vl[i]])
        

def main():
    vsub = sc_node()
    rospy.init_node('scan_values')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 