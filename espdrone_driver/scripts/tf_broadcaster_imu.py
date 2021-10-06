#!/usr/bin/python3
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import logging
from sensor_msgs.msg import Temperature, Imu
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def handle_imu_pose(msg):
    rospy.loginfo(msg)
    br = tf2_ros.TransformBroadcaster()
    
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world_frame"
    t.child_frame_id = "map"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w

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
            t.header.frame_id = "world_frame"
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
            t.child_frame_id = "/" + str(robot) + "_tf/base_link"
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