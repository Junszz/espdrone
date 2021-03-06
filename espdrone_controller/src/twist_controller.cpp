// espdrone controller
// end goal is to pub to <link>/wrench 

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <espdrone_msgs/MotorCommand.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <espdrone_controller/pid.h>

#include <limits>

class TwistController 
{
private:
  ros::NodeHandle nh;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber cmd_vel_subscriber_;
  ros::Subscriber takeoff_subscriber_;
  ros::Subscriber land_subscriber_;
  ros::Publisher wrench_pub;

  geometry_msgs::TwistStamped command_;
  geometry_msgs::WrenchStamped wrench_;
  geometry_msgs::TwistStamped twist_;
  espdrone_msgs::MotorCommand motor_;
  sensor_msgs::Imu pose_;
  sensor_msgs::Imu acceleration_;
  const ros::Duration period;
  double last_executed_time;
  double time_interval;
  double current_time;

  struct {
    struct {
      espdrone_controller::PID x;
      espdrone_controller::PID y;
      espdrone_controller::PID z;
    } linear, angular;
  } pid_;

  geometry_msgs::Wrench limits_;
  bool auto_engage_;
  double load_factor_limit;
  double mass_;
  double inertia_[3];

  bool motors_running_ = false;
  double linear_z_control_error_;
  std::string base_link_frame_, drone_index;
  std::string takeoff_topic_;
  std::string land_topic_;
  const double gravity = 9.8065;    

public:
  TwistController(): nh("~")
  { 
    // init params (only once during startup)
    // inertia = link->GetInertial()->PrincipalMoments();
    // mass = link->GetInertial()->Mass();
    mass_ = 0.5; // 26 grams
    inertia_[0] = 0.000006;
    inertia_[1] = 0.000006;
    inertia_[2] = 0.000011;
    nh.param<std::string>("base_link_frame", base_link_frame_,"base_link");
    nh.param<std::string>("drone_index", drone_index,"1");
    
    // wrench publisher
    wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/drone" + drone_index + "/wrench", 1);
    ROS_INFO("Twist_controller loaded....");
    // initialize PID controllers
    pid_.linear.x.init(ros::NodeHandle(nh, "linear/xy"));
    pid_.linear.y.init(ros::NodeHandle(nh, "linear/xy"));
    pid_.linear.z.init(ros::NodeHandle(nh, "linear/z"));
    pid_.angular.x.init(ros::NodeHandle(nh, "angular/xy"));
    pid_.angular.y.init(ros::NodeHandle(nh, "angular/xy"));
    pid_.angular.z.init(ros::NodeHandle(nh, "angular/z"));

    // load other parameters
    nh.getParam("auto_engage", auto_engage_ = true);
    nh.getParam("limits/load_factor", load_factor_limit = 1.5);
    nh.getParam("limits/force/z", limits_.force.z);
    nh.getParam("limits/torque/xy", limits_.torque.x);
    nh.getParam("limits/torque/xy", limits_.torque.y);
    nh.getParam("limits/torque/z", limits_.torque.z);
    ROS_INFO_NAMED("twist_controller", "Getting params!");

    // subscribe to pose, twist and cmd_vel
    twist_subscriber_ = nh.subscribe("/gazebo/model_states", 1, &TwistController::twistCommandCallback, this);
    // ros::Subscriber model_states_subscriber = n.subscribe("/gazebo/model_states", 100, model_states_callback);
    pose_subscriber_ = nh.subscribe<sensor_msgs::Imu>("/drone" + drone_index + "/imu", 1, &TwistController::poseCommandCallback, this);
    cmd_vel_subscriber_ = nh.subscribe<geometry_msgs::Twist>("/drone" + drone_index + "/cmd_vel", 1, &TwistController::cmd_velCommandCallback, this);
    takeoff_subscriber_ = nh.subscribe<std_msgs::Empty>("/drone" + drone_index + "/take_off", 1, &TwistController::TakeoffCallback, this);
    land_subscriber_ = nh.subscribe<std_msgs::Empty>("/drone" + drone_index + "/land", 1, &TwistController::LandCallback, this);
    last_executed_time = ros::Time::now().toSec();
  }

/*******************************************************************************
* Callback function for twist command
*******************************************************************************/
  void twistCommandCallback(gazebo_msgs::ModelStates msg)
  {
    twist_.twist = msg.twist[9];
    twist_.header.stamp = ros::Time::now();

  }
  
/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
  void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr& vel)
  {
    command_.twist = *vel; // from cmd_vel
    command_.header.stamp = ros::Time::now();
    // ROS_WARN("in call back: %f", command_.twist.linear.z);
    // ROS_INFO("command_z: %f", command_.twist.linear.z); 
  }
  
/*******************************************************************************
* Callback function for pose msg
*******************************************************************************/
  void poseCommandCallback(const sensor_msgs::Imu::ConstPtr& pose_msg)
  {
    pose_.orientation.x = pose_msg->orientation.x;
    pose_.orientation.y = pose_msg->orientation.y;
    pose_.orientation.z = pose_msg->orientation.z;
    pose_.orientation.w = pose_msg->orientation.w;
    acceleration_.linear_acceleration.x = pose_msg->linear_acceleration.x;
    acceleration_.linear_acceleration.y = pose_msg->linear_acceleration.y;
    acceleration_.linear_acceleration.z = pose_msg->linear_acceleration.z;
    update();
  }

/*******************************************************************************
* Callback function for takeoff
*******************************************************************************/
  void TakeoffCallback(const std_msgs::EmptyConstPtr& msg)
  {
    ROS_INFO("%s","\nQuadrotor takes off!!");
    motors_running_ = true;
  }

/*******************************************************************************
* Callback function for landing
*******************************************************************************/
  void LandCallback(const std_msgs::EmptyConstPtr& msg)
  {
    ROS_INFO("%s","\nQuadrotor landing!!");
    motors_running_ = false;
  }

  void update()
  {
    //get current twist
    geometry_msgs::Twist twist = twist_.twist; //current twist
    geometry_msgs::Twist twist_body;
    twist_body.linear =  toBody(twist_.twist.linear);
    twist_body.angular = toBody(twist_.twist.angular);

    geometry_msgs::Twist command = this -> command_.twist;
    // ROS_WARN("in call back: %f", command_.twist.linear.z);
    // ROS_WARN("in call back command.linear.z: %f", command.linear.z);
    // ROS_INFO("command_z_inpose: %f", command.acceleration_commandlinear.z); 

    // Transform to world coordinates if necessary (yaw only)
    double yaw = getYaw();
    geometry_msgs::Twist transformed;
    //command is from cmd_vel
    transformed.linear.x  = cos(yaw) * command.linear.x  - sin(yaw) * command.linear.y;
    transformed.linear.y  = sin(yaw) * command.linear.x  + cos(yaw) * command.linear.y;
    transformed.linear.z  = command.linear.z;
    transformed.angular.x = cos(yaw) * command.angular.x - sin(yaw) * command.angular.y;
    transformed.angular.y = sin(yaw) * command.angular.x + cos(yaw) * command.angular.y;
    transformed.angular.z  = command.angular.z;

    command = transformed;

    // calculation of load factor
    double load_factor = 1. / (  pose_.orientation.w * pose_.orientation.w
                                 - pose_.orientation.x * pose_.orientation.x
                                 - pose_.orientation.y * pose_.orientation.y
                                 + pose_.orientation.z * pose_.orientation.z );
 
    // Note: load_factor could be NaN or Inf...?
    if (load_factor_limit > 0.0 && !(load_factor < load_factor_limit)) load_factor = load_factor_limit;

    // ROS_INFO("load_factor: %f", load_factor);

    //Auto engage/shutdown
    if (auto_engage_) {
      if (!motors_running_ && command.linear.z > 0.1 && load_factor > 0.0) {
        motors_running_ = true;
        ROS_INFO_NAMED("twist_controller", "Engaging motors!");
      } else if (motors_running_ && command.linear.z < -0.1 /* && (twist.linear.z > -0.1 && twist.linear.z < 0.1) */) {
        double shutdown_limit = 1.0 * std::min(command.linear.z, -0.5);
        
        if (linear_z_control_error_ > 0.0) linear_z_control_error_ = 0.0; // positive control errors should not affect shutdown
        
        if (pid_.linear.z.getFilteredControlError(linear_z_control_error_, 5.0, time_interval) < shutdown_limit) {
          motors_running_ = false;
          ROS_INFO_NAMED("twist_controller", "Shutting down motors!");
        } else {
          ROS_DEBUG_STREAM_NAMED("twist_controller", "z control error = " << linear_z_control_error_ << " >= " << shutdown_limit);
        }
      } else {
        linear_z_control_error_ = 0.0;
      }

    //flip over?
      if (motors_running_ && load_factor < 0.0) {
        motors_running_ = false;
        ROS_WARN_NAMED("twist_controller", "Shutting down motors due to flip over! If drone stays upside down, holding key [3] until it flipped successfully");
        reset();
      }
    }

    // Update output
    if (motors_running_) {
      current_time = ros::Time::now().toSec();
      time_interval = current_time - last_executed_time;
      last_executed_time = current_time;

      geometry_msgs::Vector3 acceleration_command;
      acceleration_command.x = pid_.linear.x.update(command.linear.x, twist_.twist.linear.x, acceleration_.linear_acceleration.x, time_interval);
      acceleration_command.y = pid_.linear.y.update(command.linear.y, twist_.twist.linear.y, acceleration_.linear_acceleration.y, time_interval);
      acceleration_command.z = pid_.linear.z.update(command.linear.z, twist_.twist.linear.z, acceleration_.linear_acceleration.z, time_interval);
      geometry_msgs::Vector3 acceleration_command_body = toBody(acceleration_command);
      // ROS_DEBUG_STREAM_NAMED("twist_controller", "twist.linear:               [" << twist_.twist.linear.x << " " << twist_.twist.linear.y << " " << twist_.twist.linear.z << "]");
      // ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_body.angular:         [" << twist_body.angular.x << " " << twist_body.angular.y << " " << twist_body.angular.z << "]");
      // ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_command.linear:       [" << command.linear.x << " " << command.linear.y << " " << command.linear.z << "]");
      // ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_command.angular:      [" << command.angular.x << " " << command.angular.y << " " << command.angular.z << "]");
      // ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration:               [" << acceleration_.linear_acceleration.x << " " << acceleration_.linear_acceleration.y << " " << acceleration_.linear_acceleration.z<< "]");
      // ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration_command_world: [" << acceleration_command.x << " " << acceleration_command.y << " " << acceleration_command.z << "]");
      // ROS_WARN("current: %f, input: %f, acceleration_command_z: %f ",twist_.twist.linear.x,command.linear.z, acceleration_command.z);
      // ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration_command_body:  [" << acceleration_command_body.x << " " << acceleration_command_body.y << " " << acceleration_command_body.z << "]");

      wrench_.wrench.torque.x = inertia_[0] * pid_.angular.x.update(-acceleration_command_body.y / gravity, 0.0, twist_body.angular.x, time_interval);
      wrench_.wrench.torque.y = inertia_[1] * pid_.angular.y.update( acceleration_command_body.x / gravity, 0.0, twist_body.angular.y, time_interval);
      wrench_.wrench.torque.z = inertia_[2] * pid_.angular.z.update( command.angular.z, twist_.twist.angular.z, 0.0, time_interval);
      wrench_.wrench.force.x  = 0.0;
      wrench_.wrench.force.y  = 0.0;
      wrench_.wrench.force.z  = mass_ * (acceleration_command.z * load_factor + gravity);
      

      if (limits_.force.z > 0.0 && wrench_.wrench.force.z > limits_.force.z) wrench_.wrench.force.z = limits_.force.z;
      if (wrench_.wrench.force.z <= std::numeric_limits<double>::min()) wrench_.wrench.force.z = std::numeric_limits<double>::min();
      if (limits_.torque.x > 0.0) {
        if (wrench_.wrench.torque.x >  limits_.torque.x) wrench_.wrench.torque.x =  limits_.torque.x;
        if (wrench_.wrench.torque.x < -limits_.torque.x) wrench_.wrench.torque.x = -limits_.torque.x;
      }
      if (limits_.torque.y > 0.0) {
        if (wrench_.wrench.torque.y >  limits_.torque.y) wrench_.wrench.torque.y =  limits_.torque.y;
        if (wrench_.wrench.torque.y < -limits_.torque.y) wrench_.wrench.torque.y = -limits_.torque.y;
      }
      if (limits_.torque.z > 0.0) {
        if (wrench_.wrench.torque.z >  limits_.torque.z) wrench_.wrench.torque.z =  limits_.torque.z;
        if (wrench_.wrench.torque.z < -limits_.torque.z) wrench_.wrench.torque.z = -limits_.torque.z;
      }

      // ROS_DEBUG_STREAM_NAMED("twist_controller", "wrench_command.force:       [" << wrench_.wrench.force.x << " " << wrench_.wrench.force.y << " " << wrench_.wrench.force.z << "]");
      // ROS_DEBUG_STREAM_NAMED("twist_controller", "wrench_command.torque:      [" << wrench_.wrench.torque.x << " " << wrench_.wrench.torque.y << " " << wrench_.wrench.torque.z << "]");
      
      // ROS_INFO_NAMED("twist_controller", "controller running!");
      wrench_pub.publish(wrench_);
    } 
  }

  void reset()
  {
    pid_.linear.x.reset();
    pid_.linear.y.reset();
    pid_.linear.z.reset();
    pid_.angular.x.reset();
    pid_.angular.y.reset();
    pid_.angular.z.reset();

    wrench_.wrench.force.x  = 0.0;
    wrench_.wrench.force.y  = 0.0;
    wrench_.wrench.force.z  = 0.0;
    wrench_.wrench.torque.x = 0.0;
    wrench_.wrench.torque.y = 0.0;
    wrench_.wrench.torque.z = 0.0;
    wrench_pub.publish(wrench_);

    linear_z_control_error_ = 0.0;
    motors_running_ = false;
  }

  void starting(const ros::Time &time)
  {}

  void stopping(const ros::Time &time)
  {}

  // find yaw from quartenion
  double getYaw() const
  {
    tf2::Quaternion q;
    q[0] = pose_.orientation.x;
    q[1] = pose_.orientation.y;
    q[2] = pose_.orientation.z;
    q[3] = pose_.orientation.w;
    return atan2(2.*q[0]*q[1] + 2.*q[3]*q[2], q[0]*q[0] + q[3]*q[3] - q[2]*q[2] - q[1]*q[1]);
  }

  geometry_msgs::Vector3 toBody(geometry_msgs::Vector3& nav) const
  {
    tf2::Quaternion q;
    q[0] = pose_.orientation.x;
    q[1] = pose_.orientation.y;
    q[3] = pose_.orientation.w;
    geometry_msgs::Vector3 body;
    body.x = (q[3]*q[3]+q[0]*q[0]-q[1]*q[1]-q[2]*q[2]) * nav.x + (2.*q[0]*q[1] + 2.*q[3]*q[2]) * nav.y + (2.*q[0]*q[2] - 2.*q[3]*q[2]) * nav.z;
    body.y = (2.*q[0]*q[1] - 2.*q[3]*q[2]) * nav.x + (q[3]*q[3]-q[0]*q[0]+q[1]*q[1]-q[2]*q[2]) * nav.y + (2.*q[1]*q[2] + 2.*q[3]*q[0]) * nav.z;
    body.z = (2.*q[0]*q[2] + 2.*q[3]*q[1]) * nav.x + (2.*q[1]*q[2] - 2.*q[3]*q[0]) * nav.y + (q[3]*q[3]-q[0]*q[0]-q[1]*q[1]+q[2]*q[2]) * nav.z;
    return body;
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "twist_controller");
  TwistController node;
  ros::spin();
}
