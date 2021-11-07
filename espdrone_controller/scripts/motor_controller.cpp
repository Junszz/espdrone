//remap: wrench1....wrench2 to /drone<>/<link>/wrench
//remap: wrench to /drone<>/wrench

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>

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

  geometry_msgs::WrenchStamped wrench_;
  std::string base_link_frame_;
  double lever;


public:
  MotorController(): nh("~")
  {
    nh.param<double>("lever",lever,0.05);
    nh.param<std::string>("base_link_frame", base_link_frame_,"base_link");

    wrench_pub1 = nh.advertise<geometry_msgs::Wrench>("wrench1", 1);
    wrench_pub2 = nh.advertise<geometry_msgs::Wrench>("wrench2", 1);
    wrench_pub3 = nh.advertise<geometry_msgs::Wrench>("wrench3", 1);
    wrench_pub4 = nh.advertise<geometry_msgs::Wrench>("wrench4", 1);

    drone_wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("wrench", 1, &MotorController::wrenchCommandCallback, this);
    ROS_INFO("Motor_controller loaded....");
  }

  void wrenchCommandCallback(const geometry_msgs::WrenchStampedConstPtr& command)
  {
    wrench_ = *command;
    if (wrench_.wrench.force.z > 0.0) {
      double nominal_thrust_per_motor = wrench_.wrench.force.z / 4.0;
      double force1,force2,force3,force4;
      force1 =  nominal_thrust_per_motor - wrench_.wrench.torque.y / 2.0 / lever;
      force2 =  nominal_thrust_per_motor - wrench_.wrench.torque.x / 2.0 / lever;
      force3 =  nominal_thrust_per_motor + wrench_.wrench.torque.y / 2.0 / lever;
      force4 =  nominal_thrust_per_motor + wrench_.wrench.torque.x / 2.0 / lever;

      geometry_msgs::Wrench tmp_wrench;
      tmp_wrench.force.z = force1;
      wrench_pub1.publish(tmp_wrench);
      tmp_wrench.force.z = force2;
      wrench_pub2.publish(tmp_wrench);
      tmp_wrench.force.z = force3;
      wrench_pub3.publish(tmp_wrench);
      tmp_wrench.force.z = force4;
      wrench_pub4.publish(tmp_wrench);
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