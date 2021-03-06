
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/Wrench.h>

namespace gazebo
{
  class GazeboEspdroneDriver : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _model;
      this->robot_namespace_ = "";
      if (_sdf->HasElement("robotNamespace"))
        this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

      if (!_sdf->HasElement("linkName"))
      {
        ROS_FATAL_NAMED("force plugin", "force plugin missing <linkName>, cannot proceed");
        return;
      }

      else
        this->link_name = _sdf->GetElement("linkName")->Get<std::string>();

      this->link = _model->GetLink(this->link_name);
      inertia = link->GetInertial()->PrincipalMoments();
      mass = link->GetInertial()->Mass();
      ROS_WARN("inertia: %f, %f, %f",inertia[0],inertia[1],inertia[2]);
      ROS_WARN("mass: %f ",mass);

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GazeboEspdroneDriver::OnUpdate, this));
  

      // subscribing topic name, ex drone2/FL_link_2/wrench
      std::string wrench_topic = "/" + this->robot_namespace_ + "/" + this->link_name + "/wrench";

      // Initialize ros, if it has not already been initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, this->link_name + "_motor_driver",
            ros::init_options::NoSigintHandler);
      }
         
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle(this->link_name + "_motor_driver"));
      
      // initialize subscriber to wrench topic
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Wrench>(
            wrench_topic,
            1,
            boost::bind(&GazeboEspdroneDriver::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&GazeboEspdroneDriver::QueueThread, this));
             
      ROS_WARN("Initialized motor driver");
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      //apply force to the link in vector 3d (x,y,z) format 
      this->link->AddRelativeForce(this->link_force);
      this->link->AddRelativeTorque(this->link_torque);
      //plan 2: AddLinkForce (const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_offset=ignition::math::Vector3d::Zero)

    }
    
    
    public: void OnRosMsg(const geometry_msgs::Wrench::ConstPtr &msg)
    {
      //extract force from wrench message and changed it to vector3d
      ignition::math::Vector3d force_3d(msg -> force.x, msg -> force.y, msg -> force.z);
      ignition::math::Vector3d torque_3d(msg -> torque.x, msg -> torque.y, msg -> torque.z);
      this->link_force = ignition::math::Vector3d(force_3d);
      this->link_torque = ignition::math::Vector3d(torque_3d);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: ignition::math::Vector3d inertia;
    private: double mass;
    private: ignition::math::Vector3d link_force;
    private: ignition::math::Vector3d link_torque;
    private: std::string robot_namespace_;
    /// \brief A pointer to the Gazebo joint

    private: std::string link_name;

    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboEspdroneDriver)
}