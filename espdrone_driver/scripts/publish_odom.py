#!/usr/bin/env python3


import rospy
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def converge(msg, drone_index):
    global last_time, last_x,last_y,last_z,last_R, last_P, last_Y
    time = rospy.Time.now()
    dt = time.to_sec() - last_time
    last_time = time.to_sec() 
    odometry = Odometry()
    odometry.header.stamp = time
    odometry.header.frame_id = "aruco_map"
    odometry.child_frame_id = "camera_link_" + str(drone_index)

    odometry.pose.pose.position.x = msg.pose.position.x 
    odometry.pose.pose.position.y = msg.pose.position.y
    odometry.pose.pose.position.z = msg.pose.position.z
    odometry.pose.pose.orientation.x = msg.pose.orientation.x
    odometry.pose.pose.orientation.y = msg.pose.orientation.y
    odometry.pose.pose.orientation.z = msg.pose.orientation.z
    odometry.pose.pose.orientation.w = msg.pose.orientation.w


    vx = (msg.pose.position.x - last_x)/ dt
    last_x = msg.pose.position.x 
    vy = (msg.pose.position.y - last_y)/ dt
    last_y = msg.pose.position.y
    vz = (msg.pose.position.z - last_z)/ dt
    last_z = msg.pose.position.z

    R,P,Y = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
    vR = (R - last_R)/ dt
    last_R = R
    vP = (P - last_P)/ dt
    last_P = P
    vY= (Y - last_Y)/ dt
    last_Y = Y

    odometry.twist.twist = Twist(Vector3(vx,vy,vz),Vector3(vR,vP,vY))
    odometry.pose.covariance = cov_pose
    print(odometry)
    odom_publisher.publish(odometry)


if __name__ == '__main__':

    last_time = 0
    last_x = 0
    last_y = 0
    last_z = 0
    last_R = 0
    last_Y = 0
    last_P = 0

    cov_pose = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 1, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 1, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, 1]

    rospy.init_node('odometry_publisher')
    index = rospy.get_param("~index")
    r = rospy.Rate(10) 
    odom_publisher = rospy.Publisher('/vo', Odometry, queue_size=10)
    rospy.Subscriber('/pose', PoseStamped, converge, str(index))
    rospy.spin()