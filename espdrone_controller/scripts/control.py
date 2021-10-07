#!/usr/bin/env python
#---------------------------------------------------
from pid import PID
import argparse
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64, Float32, Float64MultiArray
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
#---------------------------------------------------
def control_esp(msg, args):
	#Declare global variables as you dont want these to die, reset to zero and then re-initiate when the function is called again.
	global roll, pitch, yaw, err_roll, err_pitch, err_yaw
	
	#Assign the Float64MultiArray object to 'f' as we will have to send data of motor velocities to gazebo in this format
	f = Float64MultiArray()
	
	#Convert the quaternion data to roll, pitch, yaw data
	#The model_states contains the position, orientation, velocities of all objects in gazebo. In the simulation, there are objects like: ground, Contruction_cone, quadcopter (named as 'Kwad') etc. So 'msg.pose[ind]' will access the 'Kwad' object's pose information i.e the quadcopter's pose.
	ind = msg.name.index(drone)
	orientationObj = msg.pose[ind].orientation
	orientationList = [orientationObj.x, orientationObj.y, orientationObj.z, orientationObj.w]
	(roll, pitch, yaw) = (euler_from_quaternion(orientationList))
    #send roll, pitch, yaw data to PID() for attitude-stabilisation, along with 'f', to obtain 'fUpdated'
	#Alternatively, you can add your 'control-file' with other algorithms such as Reinforcement learning, and import the main function here instead of PID().
	(fr, fl, bl, br, err_roll, err_pitch, err_yaw) = PID(roll, pitch, yaw, f, args[7])
	
	#The object args contains the tuple of objects (velPub, err_rollPub, err_pitchPub, err_yawPub. publish the information to namespace.
	args[0].publish(fr)
	args[1].publish(fl)
	args[2].publish(bl)
	args[3].publish(br)
	args[4].publish(err_roll)
	args[5].publish(err_pitch)
	args[6].publish(err_yaw)

	#print("Roll: ",roll*(180/3.141592653),"Pitch: ", pitch*(180/3.141592653),"Yaw: ", yaw*(180/3.141592653))
	#print(orientationObj)
#----------------------------------------------------
if __name__ == '__main__':
    try:
	#Initiate the node that will control the gazebo model
	rospy.init_node("Control")
		
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

	#initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that it can be plotted via rqt_plot /err_<name>  
	err_rollPub = rospy.Publisher('err_roll', Float32, queue_size=1)
	err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)
	err_yawPub = rospy.Publisher('err_yaw', Float32, queue_size=1)

	#initialte publisher velPub that will publish the velocities of individual BLDC motors
	drone = args.drone[0]
	vel = int(args.vel)

	velPub1 = rospy.Publisher('/'+ drone + '/joint1_velocity_controller/command', Float64, queue_size=4)
	velPub2 = rospy.Publisher('/'+ drone + '/joint2_velocity_controller/command', Float64, queue_size=4)
	velPub3 = rospy.Publisher('/'+ drone + '/joint3_velocity_controller/command', Float64, queue_size=4)
	velPub4 = rospy.Publisher('/'+ drone + '/joint4_velocity_controller/command', Float64, queue_size=4)

	#Subscribe to /gazebo/model_states to obtain the pose in quaternion form
	#Upon receiveing the messages, the objects msg, velPub, err_rollPub, err_pitchPub and err_yawPub are sent to "control_kwad" function.
	PoseSub = rospy.Subscriber('/gazebo/model_states',ModelStates,control_esp,(velPub1, velPub2, velPub3, velPub4, err_rollPub, err_pitchPub, err_yawPub, vel))

	rospy.spin()

    except rospy.ROSInterruptException:
        pass
