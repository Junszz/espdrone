#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64

def talker():
    pub1 = rospy.Publisher('/drone2/joint1_velocity_controller/command', Float64, queue_size=10)
 
    rospy.init_node('vel_publisher', anonymous=True)
    print("publishing")
    pub1.publish(1000)


    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass