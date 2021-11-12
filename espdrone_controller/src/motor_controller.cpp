//remap: wrench1....wrench2 to /drone<>/<link>/wrench
//remap: wrench to /drone<>/wrench

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

class MotorController
{
private:
  ros::NodeHandle nh;

  ros::Subscriber drone_wrench_sub;

  ros::Publisher wrench_pub1;
  ros::Publisher wrench_pub2;
  ros::Publisher wrench_pub3;
  ros::Publisher wrench_pub4;

  ros::Publisher vel_pub1;
  ros::Publisher vel_pub2;
  ros::Publisher vel_pub3;
  ros::Publisher vel_pub4;

  geometry_msgs::WrenchStamped wrench_;
  std::string base_link_frame_;
  std::string drone_index;
  double lever;

  double coefficient = 480e-9;
  double force1,force2,force3,force4;
  std_msgs::Float64 vel1,vel2,vel3,vel4;
  geometry_msgs::Wrench tmp_wrench;


// M1          M2
// O           O


// M4          M3
// O           O


public:
  MotorController(): nh("~")
  {
    nh.param<double>("lever",lever,0.05);
    nh.param<std::string>("base_link_frame", base_link_frame_,"base_link");
    nh.param<std::string>("drone_index", drone_index,"1");

    wrench_pub1 = nh.advertise<geometry_msgs::Wrench>("/drone" + drone_index + "/FL_link_" + drone_index + "/wrench", 1);
    wrench_pub2 = nh.advertise<geometry_msgs::Wrench>("/drone" + drone_index + "/FR_link_" + drone_index + "/wrench", 1);
    wrench_pub3 = nh.advertise<geometry_msgs::Wrench>("/drone" + drone_index + "/BR_link_" + drone_index + "/wrench", 1);
    wrench_pub4 = nh.advertise<geometry_msgs::Wrench>("/drone" + drone_index + "/BL_link_" + drone_index + "/wrench", 1);

    vel_pub1 = nh.advertise<std_msgs::Float64>("/drone" + drone_index + "/jointFL_velocity_controller/command", 1);
    vel_pub2 = nh.advertise<std_msgs::Float64>("/drone" + drone_index + "/jointFR_velocity_controller/command", 1);
    vel_pub3 = nh.advertise<std_msgs::Float64>("/drone" + drone_index + "/jointBR_velocity_controller/command", 1);
    vel_pub4 = nh.advertise<std_msgs::Float64>("/drone" + drone_index + "/jointBL_velocity_controller/command", 1);

    drone_wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/drone" + drone_index + "/wrench", 1, &MotorController::wrenchCommandCallback, this);
    ROS_INFO("Motor_controller loaded....");
  }

  void wrenchCommandCallback(const geometry_msgs::WrenchStampedConstPtr& command)
  {
    wrench_ = *command;
    if (wrench_.wrench.force.z > 0.0) {
      double nominal_thrust_per_motor = wrench_.wrench.force.z / 4.0;

      // force1 =  nominal_thrust_per_motor + wrench_.wrench.torque.x / 4.0 / lever + wrench_.wrench.torque.y / 4.0 / lever;
      // vel1.data = sqrt(force1/coefficient);
      // force2 =  nominal_thrust_per_motor + wrench_.wrench.torque.x / 4.0 / lever - wrench_.wrench.torque.y / 4.0 / lever;
      // vel2.data = sqrt(force2/coefficient);
      // force3 =  nominal_thrust_per_motor - wrench_.wrench.torque.x / 4.0 / lever - wrench_.wrench.torque.y / 4.0 / lever;
      // vel3.data = sqrt(force3/coefficient);
      // force4 =  nominal_thrust_per_motor - wrench_.wrench.torque.x / 4.0 / lever + wrench_.wrench.torque.y / 4.0 / lever;
      // vel4.data = sqrt(force4/coefficient);

      force1 =  nominal_thrust_per_motor + wrench_.wrench.torque.y / 2.0 / lever;
      vel1.data = sqrt(force1/coefficient);

      force2 =  nominal_thrust_per_motor + wrench_.wrench.torque.x / 2.0 / lever;
      vel2.data = sqrt(force1/coefficient);

      force3 =  nominal_thrust_per_motor - wrench_.wrench.torque.y / 2.0 / lever;
      vel3.data = sqrt(force3/coefficient);

      force4 =  nominal_thrust_per_motor - wrench_.wrench.torque.x / 2.0 / lever;
      vel4.data = sqrt(force4/coefficient);

      tmp_wrench.force.z = force1;
      wrench_pub1.publish(tmp_wrench);
      tmp_wrench.force.z = force2;
      wrench_pub2.publish(tmp_wrench);
      tmp_wrench.force.z = force3;
      wrench_pub3.publish(tmp_wrench);
      tmp_wrench.force.z = force4;
      wrench_pub4.publish(tmp_wrench);
      
      vel_pub1.publish(vel1);
      vel_pub2.publish(vel2);
      vel_pub3.publish(vel3);
      vel_pub4.publish(vel4);

    } else {
      wrench_.wrench.force.x  = 0.0;
      wrench_.wrench.force.y  = 0.0;
      wrench_.wrench.force.z  = 0.0;
      wrench_.wrench.torque.x = 0.0;
      wrench_.wrench.torque.y = 0.0;
      wrench_.wrench.torque.z = 0.0;
    }
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motor_controller");
  MotorController node;
  ros::spin();
}