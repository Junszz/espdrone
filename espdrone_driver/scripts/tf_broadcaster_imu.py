#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Temperature, Imu
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import argparse


dif_camera_base_x = 0.02
dif_camera_base_y = 0
dif_camera_base_z = 0

def handle_vo_pose(msg,drone_index):
    x = msg.pose.position.x  - dif_camera_base_x
    y = msg.pose.position.y - dif_camera_base_y
    z = msg.pose.position.z - dif_camera_base_z
    x_o, y_o, z_o, w = msg.pose.orientation

    (roll, pitch, yaw) = euler_from_quaternion([x_o,y_o,z_o,w])    
    q1 = quaternion_from_euler(0.0, 0.0, yaw)                       #orientation transform for base_footprint
    q2 = quaternion_from_euler(roll, pitch, 0)                      #orientation transform for base_link
    
    br_base = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "aruco_map"
    t.child_frame_id = "base_footprint_"+ drone_index
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0
    t.transform.rotation.x = q1[0]
    t.transform.rotation.y = q1[1]
    t.transform.rotation.z = q1[2]
    t.transform.rotation.w = q1[3]
    br_base.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_footprint_"+ drone_index
    t.child_frame_id = "base_link_" + drone_index
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = z
    t.transform.rotation.x = q2[0]
    t.transform.rotation.y = q2[1]
    t.transform.rotation.z = q2[2]
    t.transform.rotation.w = q2[3]
    br.sendTransform(t)

def handle_gazebo_pose(msg):
    global robots
    for drone_index,robot in enumerate(list(msg.name)):
        if str(robot) in robots:
            x = msg.pose[drone_index].position.x
            y = msg.pose[drone_index].position.y
            z = msg.pose[drone_index].position.z
            x_o = msg.pose[drone_index].orientation.x
            y_o = msg.pose[drone_index].orientation.y
            z_o = msg.pose[drone_index].orientation.z
            w = msg.pose[drone_index].orientation.w

            (roll, pitch, yaw) = euler_from_quaternion([x_o,y_o,z_o,w])    
            q1 = quaternion_from_euler(0.0, 0.0, yaw)                       #orientation transform for base_footprint
            q2 = quaternion_from_euler(roll, pitch, 0)                      #orientation transform for base_link
            
            br_base = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "aruco_map"
            t.child_frame_id = "/" + str(robot) + "_tf/base_footprint"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0
            t.transform.rotation.x = q1[0]
            t.transform.rotation.y = q1[1]
            t.transform.rotation.z = q1[2]
            t.transform.rotation.w = q1[3]
            br_base.sendTransform(t)

            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "/" + str(robot) + "_tf/base_footprint"
            t.child_frame_id = "base_link_" + str(robot)[-1]
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = z
            t.transform.rotation.x = q2[0]
            t.transform.rotation.y = q2[1]
            t.transform.rotation.z = q2[2]
            t.transform.rotation.w = q2[3]
            br.sendTransform(t)

if __name__ == '__main__':

    global drone_index
    global robots
    drone_index = None
    robots = ["drone1","drone2","drone3"]
    rospy.loginfo("initialize node...")
    rospy.init_node('tf_broadcaster_imu')
    rospy.loginfo("listening...")
    #rospy.Subscriber('/imu', Imu, handle_imu_pose)
    rospy.Subscriber('/gazebo/model_states', ModelStates, handle_gazebo_pose)
    rospy.spin()
    '''
    parser = argparse.ArgumentParser(
                description="publish velocity to motors"
            )
    parser.add_argument(
        "--index",
        "-i",
        required=True,
        help="drone_index",
    )

    args = parser.parse_args() 
    rospy.spin()
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/pose', Pose, handle_vo_pose, args.index)
    rospy.spin()
        '''