# ROS-Espdrone-Simulation
This is an academic project on simulating a drone and perform various features such as keyboard_teleop and hovering on ROS-Gazebo(noetic version of ROS)

This is a simulation of ESP-drone mini Quadcopter. For more information please refer to:

Github repo:


The Model of the drone is designed in Solidwords and written in .xacro format, then imported into the Gazebo simulator.

All the mass, moment of inertia etc are calibrated and is identical to the real ESP-drone frame.

Any progress/add-ons will be updated in this reposity.

# Installation #

Make sure you've installed ROS and Gazebo on your systems. A docker container with ROS environment is required to run this project on a Windows machine.

Open a terminal.
1. Initiate a workspace in your home directory or use your existing favorite one.
```
source /opt/ros/noetic/setup.bash 

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

2. Create necessary workspace files
```
cd ~/catkin_ws
catkin_make
```

3. Add this workspace to your linux environment by sourcing the setup file to .bashrc. Assuming you are inside the home directory, 
```
cd ~
gedit .bashrc
```
Add this line at the end of the file.
```
source ~/catkin_ws/devel/setup.bash
```

4. Clone all the folders and files in this repo into /src

Then make the python scripts executable by,
```
cd ~/catkin_ws/src/espdrone/espdrone_driver/scripts/
chmod +x tf_broadcaster_imu.py
cd ../..
cd ~/catkin_ws/src/espdrone/espdrone_controller/scripts/
chmod +x keyboard_teleop.py

```

5. Execute the following command to build into your ROS workspace
```
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

This should build the directory without any errors. If you find any errors, please check your steps with those mentioned here.

Once installed, close the terminal. Open another terminal and load the quadcopter into gazebo simulator
```
roslaunch espdrone_driver main.launch
```

This should load the Quadcopter.

![Image](https://github.com/Junszz/espdrone/blob/main/Image.png)

The pid values are in the espdrone_controller/config/espdrone.yaml file in your espdrone directory. Play around with the values to see some control theory in action!