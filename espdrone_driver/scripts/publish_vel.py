#!/usr/bin/python3

import rospy
import argparse
from std_msgs.msg import Float64

def talker(drone, vel):
    rospy.init_node('vel_publisher', anonymous=True)
    pub1 = rospy.Publisher('/' + drone + '/joint1_velocity_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/' + drone + '/joint2_velocity_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/' + drone + '/joint3_velocity_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/' + drone + '/joint4_velocity_controller/command', Float64, queue_size=10)

    print(f"publishing...vel -> {vel} to {drone}")

    pub1.publish(int(vel))
    pub2.publish(int(vel))
    pub3.publish(int(vel))
    pub4.publish(int(vel))
    rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(
            description="publish velocity to motors"
        )
        parser.add_argument(
            "--drone",
            "-d",
            type=str,
            nargs="+",
            required=True,
            metavar="<drone_names>",
            help="Drone(s) to launch, exclude '.yaml'",
        )
        parser.add_argument(
            "--vel",
            "-v",
            type=int,
            required=True,
            metavar="<env_name>",
            help="Environment (map) with ArUco markers to fly in, exclude '.yaml'",
        )
        args = parser.parse_args()
        while(1):
            talker(args.drone[0], int(args.vel))
    except rospy.ROSInterruptException:
        pass