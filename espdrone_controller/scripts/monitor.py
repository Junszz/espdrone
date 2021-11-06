import matplotlib
import tkinter
# matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading

def clear_plt():
    global axs
    axs[0, 0].clear()
    axs[1, 0].clear()
    axs[2, 0].clear()
    axs[0, 1].clear()
    axs[1, 1].clear()
    axs[2, 1].clear()
    axs[0, 0].set_title('X')
    axs[1, 0].set_title('Y')
    axs[2, 0].set_title('Z')
    axs[0, 1].set_title('Roll')
    axs[1, 1].set_title('Pitch')
    axs[2, 1].set_title('Yaw')

def initialize_arr():
    global arr_x,arr_y,arr_z,arr_R,arr_P,arr_Y, last_time, x_range
    arr_x = arr_y = arr_z = arr_R = arr_P = arr_Y = np.zeros(x_range)
    last_time = 0 

def update(msg):
    global axs,arr_x,arr_y,arr_z,arr_R,arr_P,arr_Y, last_time, update_period, thread_into
    time = msg.header.stamp
    time_in_sec = time.to_sec()
    if (time_in_sec - last_time) > update_period:
        print("in update")
        last_time = time_in_sec
        x,y,z = msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z
        R,P,Y = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        arr_x = np.append(arr_x,x)[1:]
        arr_y = np.append(arr_y,y)[1:]
        arr_z = np.append(arr_z,z)[1:]
        arr_R = np.append(arr_R,R)[1:]
        arr_P = np.append(arr_P,P)[1:]
        arr_Y = np.append(arr_Y,Y)[1:]
        listener()

        # if not thread_into:
        #     plotting()
        #     thread_into = 1
        # clear_plt()
        # axs[0,0].plot(arr_x)
        # axs[1,0].plot(arr_y)
        # axs[2,0].plot(arr_z)
        # axs[0,1].plot(arr_R)
        # axs[1,1].plot(arr_P)
        # axs[2,1].plot(arr_Y)
        # plt.show(block=True)
        #fig.savefig("mygraph.png",bbox_inches='tight')
        

def plotting():
    global axs,arr_x,arr_y,arr_z,arr_R,arr_P,arr_Y, update_period,T
    print("in plot")
    clear_plt()
    axs[0,0].plot(arr_x)
    axs[1,0].plot(arr_y)
    axs[2,0].plot(arr_z)
    axs[0,1].plot(arr_R)
    axs[1,1].plot(arr_P)
    axs[2,1].plot(arr_Y)
    plt.show()

    # if T is not None:
    #     T.cancel()
    # T = threading.Timer(update_period, plotting)
    # T.start()

def listener():
    global sub
    sub = rospy.Subscriber("/drone1/camera1/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, update)
    print("in_listen")
    plotting()

    
if __name__=="__main__":
    thread_into=0
    T=None
    x_range = 50
    update_period = 0.5
    last_time=0
    rospy.init_node('pose_monitor', anonymous=False)
    rospy.loginfo("start drone's position monitoring") 
    fig, axs = plt.subplots(3, 2)
    initialize_arr()
    # topic_name = rospy.get_param("~topic")
    # x_range = rospy.get_param("~axisRange",50)
    # update_period = rospy.get_param("~updatePeriod",0.1)
    listener()


    