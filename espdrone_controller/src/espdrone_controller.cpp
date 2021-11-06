// espdrone controller
// end goal is to pub to <link>/wrench 

#include <espdrone_controller/pid.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <limits>

namespace espdrone_controller {

class TwistController : public controller_twist
{
public:
  TwistController()
  {}

  ~TwistController()
  {}

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    // get init params (only once during startup)
    goal_linear_velocity = 0.0;
    goal_angular_velocity = 0.0;
    // pose consist of x_ref, y_ref, z_ref
    mass_ = 0.026 //26 grams
    // inertia in the form of ixx, iyy, izz
    inertia_[3] = [5.69029262704911E-06, 5.38757483059318E-06, 1.04978709710599E-05]
    pose_ =  


    // subscribe to commanded twist (geometry_msgs/TwistStamped) and cmd_vel (geometry_msgs/Twist)
    twist_subscriber_ = node_handle_.subscribe<geometry_msgs::TwistStamped>("command/twist", 1, boost::bind(&TwistController::twistCommandCallback, this, _1));
    cmd_vel_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&TwistController::cmd_velCommandCallback, this, _1));
    
    // engage/shutdown service servers
    engage_service_server_ = node_handle_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("engage", boost::bind(&TwistController::engageCallback, this, _1, _2));
    shutdown_service_server_ = node_handle_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("shutdown", boost::bind(&TwistController::shutdownCallback, this, _1, _2));

    // initialize PID controllers
    pid_.linear.x.init(ros::NodeHandle(controller_nh, "linear/xy"));
    pid_.linear.y.init(ros::NodeHandle(controller_nh, "linear/xy"));
    pid_.linear.z.init(ros::NodeHandle(controller_nh, "linear/z"));
    pid_.angular.x.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular.y.init(ros::NodeHandle(controller_nh, "angular/xy"));
    pid_.angular.z.init(ros::NodeHandle(controller_nh, "angular/z"));

    // load other parameters
    controller_nh.getParam("auto_engage", auto_engage_ = true);
    controller_nh.getParam("limits/load_factor", load_factor_limit = 1.5);
    controller_nh.getParam("limits/force/z", limits_.force.z);
    controller_nh.getParam("limits/torque/xy", limits_.torque.x);
    controller_nh.getParam("limits/torque/xy", limits_.torque.y);
    controller_nh.getParam("limits/torque/z", limits_.torque.z);
    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

    // equate mass = 26g if necessary

    command_given_in_stabilized_frame_ = false;

    return true;
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

    linear_z_control_error_ = 0.0;
    motors_running_ = false;
  }

  void twistCommandCallback(const geometry_msgs::TwistStampedConstPtr& command)
  {
    // mutex is used to avoid race condition
    // lock(), unlock()
    boost::mutex::scoped_lock lock(command_mutex_);

    command_ = *command;
    if (command_.header.stamp.isZero()) command_.header.stamp = ros::Time::now();
    command_given_in_stabilized_frame_ = false;

    // start controller if it not running
    if (!isRunning()) this->startRequest(command_.header.stamp);
  }
    
  void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr& command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    command_.twist = *command;
    command_.header.stamp = ros::Time::now();
    goal_linear_velocity = command_.twist->linear.x;
    goal_angular_velocity = command_.twiss->angular.z;

    command_given_in_stabilized_frame_ = true;

    // start controller if it not running
    if (!isRunning()) this->startRequest(command_.header.stamp);
  }
  
  bool engageCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ROS_INFO_NAMED("twist_controller", "Engaging motors!");
    motors_running_ = true;
    return true;
  }

  bool shutdownCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ROS_INFO_NAMED("twist_controller", "Shutting down motors!");
    motors_running_ = false;
    return true;
  }

    void starting(const ros::Time &time)
  {
    reset();
    wrench_output_->start();
  }

  void stopping(const ros::Time &time)
  {
    wrench_output_->stop();
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    // Get twist command input
    
    if (twist_input_->connected() && twist_input_->enabled()) {
      // replace getcommand directly with linear and angular values
      command_.twist = twist_input_->getCommand();
      command_given_in_stabilized_frame_ = false;
    }

    // Get current state and command
    Twist command = command_.twist;
    Twist twist = twist_->twist();
    Twist twist_body;
    twist_body.linear =  pose_->toBody(twist.linear);
    twist_body.angular = pose_->toBody(twist.angular);

    // Transform to world coordinates if necessary (yaw only)
    if (command_given_in_stabilized_frame_) {
      double yaw = pose_->getYaw();
      Twist transformed = command;
      transformed.linear.x  = cos(yaw) * command.linear.x  - sin(yaw) * command.linear.y;
      transformed.linear.y  = sin(yaw) * command.linear.x  + cos(yaw) * command.linear.y;
      transformed.angular.x = cos(yaw) * command.angular.x - sin(yaw) * command.angular.y;
      transformed.angular.y = sin(yaw) * command.angular.x + cos(yaw) * command.angular.y;
      command = transformed;
    }

    // Get gravity and load factor
    // load factor is equal to 1 when the aircraft is static on the ground
    const double gravity = 9.8065;

    // calculation of load factor
    double load_factor = 1. / (  pose_->pose().orientation.w * pose_->pose().orientation.w
                                 - pose_->pose().orientation.x * pose_->pose().orientation.x
                                 - pose_->pose().orientation.y * pose_->pose().orientation.y
                                 + pose_->pose().orientation.z * pose_->pose().orientation.z );
    // Note: load_factor could be NaN or Inf...?
    if (load_factor_limit > 0.0 && !(load_factor < load_factor_limit)) load_factor = load_factor_limit;

    // Auto engage/shutdown
    if (auto_engage_) {
      if (!motors_running_ && command.linear.z > 0.1 && load_factor > 0.0) {
        motors_running_ = true;
        ROS_INFO_NAMED("twist_controller", "Engaging motors!");
      } else if (motors_running_ && command.linear.z < -0.1 /* && (twist.linear.z > -0.1 && twist.linear.z < 0.1) */) {
        double shutdown_limit = 0.25 * std::min(command.linear.z, -0.5);
        if (linear_z_control_error_ > 0.0) linear_z_control_error_ = 0.0; // positive control errors should not affect shutdown
        if (pid_.linear.z.getFilteredControlError(linear_z_control_error_, 5.0, period) < shutdown_limit) {
          motors_running_ = false;
          ROS_INFO_NAMED("twist_controller", "Shutting down motors!");
        } else {
          ROS_DEBUG_STREAM_NAMED("twist_controller", "z control error = " << linear_z_control_error_ << " >= " << shutdown_limit);
        }
      } else {
        linear_z_control_error_ = 0.0;
      }

      // flip over?
      if (motors_running_ && load_factor < 0.0) {
        motors_running_ = false;
        ROS_WARN_NAMED("twist_controller", "Shutting down motors due to flip over!");
      }
    }

  //publisher to publish output

    // Update output
    if (motors_running_) {
      Vector3 acceleration_command;
      acceleration_command.x = pid_.linear.x.update(command.linear.x, twist.linear.x, acceleration_->acceleration().x, period);
      acceleration_command.y = pid_.linear.y.update(command.linear.y, twist.linear.y, acceleration_->acceleration().y, period);
      acceleration_command.z = pid_.linear.z.update(command.linear.z, twist.linear.z, acceleration_->acceleration().z, period) + gravity;
      Vector3 acceleration_command_body = pose_->toBody(acceleration_command);

      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist.linear:               [" << twist.linear.x << " " << twist.linear.y << " " << twist.linear.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_body.angular:         [" << twist_body.angular.x << " " << twist_body.angular.y << " " << twist_body.angular.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_command.linear:       [" << command.linear.x << " " << command.linear.y << " " << command.linear.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "twist_command.angular:      [" << command.angular.x << " " << command.angular.y << " " << command.angular.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration:               [" << acceleration_->acceleration().x << " " << acceleration_->acceleration().y << " " << acceleration_->acceleration().z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration_command_world: [" << acceleration_command.x << " " << acceleration_command.y << " " << acceleration_command.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "acceleration_command_body:  [" << acceleration_command_body.x << " " << acceleration_command_body.y << " " << acceleration_command_body.z << "]");

      wrench_.wrench.torque.x = inertia_[0] * pid_.angular.x.update(-acceleration_command_body.y / gravity, 0.0, twist_body.angular.x, period);
      wrench_.wrench.torque.y = inertia_[1] * pid_.angular.y.update( acceleration_command_body.x / gravity, 0.0, twist_body.angular.y, period);
      wrench_.wrench.torque.z = inertia_[2] * pid_.angular.z.update( command.angular.z, twist.angular.z, 0.0, period);
      wrench_.wrench.force.x  = 0.0;
      wrench_.wrench.force.y  = 0.0;
      wrench_.wrench.force.z  = mass_ * ((acceleration_command.z - gravity) * load_factor + gravity);

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

      ROS_DEBUG_STREAM_NAMED("twist_controller", "wrench_command.force:       [" << wrench_.wrench.force.x << " " << wrench_.wrench.force.y << " " << wrench_.wrench.force.z << "]");
      ROS_DEBUG_STREAM_NAMED("twist_controller", "wrench_command.torque:      [" << wrench_.wrench.torque.x << " " << wrench_.wrench.torque.y << " " << wrench_.wrench.torque.z << "]");

      // compute motor output 
      double nominal_thrust_per_motor = wrench_.wrench.force.z / 4.0;
      // 3 propellers so divided by 3, torque = F x l
      // set the wrench to this format /drone_$(arg drone_index)/BL_link_2/wrench
      motor_.force[0] =  nominal_thrust_per_motor - wrench_.wrench.torque.y / 3.0 / parameters_.lever;
      motor_.force[1] =  nominal_thrust_per_motor - wrench_.wrench.torque.x / 3.0 / parameters_.lever;
      motor_.force[2] =  nominal_thrust_per_motor + wrench_.wrench.torque.y / 3.0 / parameters_.lever;
      motor_.force[3] =  nominal_thrust_per_motor + wrench_.wrench.torque.x / 3.0 / parameters_.lever;

      // convert force to rpm, then pub to joint_state_controller

      // double nominal_torque_per_motor = wrench_.wrench.torque.z / 4.0;
      // motor_.voltage[0] = motor_.force[0] / parameters_.force_per_voltage + nominal_torque_per_motor / parameters_.torque_per_voltage;
      // motor_.voltage[1] = motor_.force[1] / parameters_.force_per_voltage - nominal_torque_per_motor / parameters_.torque_per_voltage;
      // motor_.voltage[2] = motor_.force[2] / parameters_.force_per_voltage + nominal_torque_per_motor / parameters_.torque_per_voltage;
      // motor_.voltage[3] = motor_.force[3] / parameters_.force_per_voltage - nominal_torque_per_motor / parameters_.torque_per_voltage;

      // motor_.torque[0] = motor_.voltage[0] * parameters_.torque_per_voltage;
      // motor_.torque[1] = motor_.voltage[1] * parameters_.torque_per_voltage;
      // motor_.torque[2] = motor_.voltage[2] * parameters_.torque_per_voltage;
      // motor_.torque[3] = motor_.voltage[3] * parameters_.torque_per_voltage;

      if (motor_.voltage[0] < 0.0) motor_.voltage[0] = 0.0;
      if (motor_.voltage[1] < 0.0) motor_.voltage[1] = 0.0;
      if (motor_.voltage[2] < 0.0) motor_.voltage[2] = 0.0;
      if (motor_.voltage[3] < 0.0) motor_.voltage[3] = 0.0;

    } else {
      reset();
    }

    // set wrench output
    wrench_.header.stamp = time;
    wrench_.header.frame_id = base_link_frame_;
    // instead of using setcommand, change to rostopic publish
    wrench_output_->setCommand(wrench_.wrench);
  }

private:
  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  AccelerationHandlePtr acceleration_;
  TwistCommandHandlePtr twist_input_;
  WrenchCommandHandlePtr wrench_output_;

  ros::NodeHandle node_handle_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber cmd_vel_subscriber_;
  ros::Subscriber imu_topic_;
  ros::ServiceServer engage_service_server_;
  ros::ServiceServer shutdown_service_server_;

  geometry_msgs::TwistStamped command_;
  geometry_msgs::WrenchStamped wrench_;
  bool command_given_in_stabilized_frame_;
  std::string base_link_frame_;

  struct {
    struct {
      PID x;
      PID y;
      PID z;
    } linear, angular;
  } pid_;

  geometry_msgs::Wrench limits_;
  bool auto_engage_;
  double load_factor_limit;
  double mass_;
  double inertia_[3];

  bool motors_running_;
  double linear_z_control_error_;
  boost::mutex command_mutex_;

};

}


}