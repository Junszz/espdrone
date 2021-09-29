#!/usr/bin/python3
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import logging
from sensor_msgs.msg import Temperature, Imu
from gazebo_msgs.msg import ModelStates

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
        if "drone" in robot:
            x = msg.pose[drone_index].position.x
            y = msg.pose[drone_index].position.y
            z = msg.pose[drone_index].position.z
            roll = msg.pose[drone_index].orientation.x
            pitch = msg.pose[drone_index].orientation.y
            yaw = msg.pose[drone_index].orientation.z
            w = msg.pose[drone_index].orientation.w

            br_base = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world_frame"
            t.child_frame_id = "/" + str(robot) + "_tf/base_footprint"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0
            t.transform.rotation.x = roll
            t.transform.rotation.y = pitch
            t.transform.rotation.z = yaw
            t.transform.rotation.w = w
            br_base.sendTransform(t)

            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "/" + str(robot) + "_tf/base_footprint"
            t.child_frame_id = "/" + str(robot) + "_tf/base_link"
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = z
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
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
