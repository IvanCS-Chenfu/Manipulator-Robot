// manipulator_torque_plugin.cpp

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <thread>

namespace gazebo
{
  class ManipulatorTorquePlugin : public ModelPlugin
  {
  public:
    ManipulatorTorquePlugin() : ModelPlugin(), inited_(false) {}

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      model_ = model;

      // --- initialize an rclcpp::Node inside this plugin ---
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      node_ = rclcpp::Node::make_shared("manipulator_torque_plugin");

      // --- get the topic name, default to /desired_torques ---
      std::string topic = "/desired_torques";
      if (sdf->HasElement("ros_topic")) {
        topic = sdf->Get<std::string>("ros_topic");
      }

      // --- subscription to the torque array ---
      sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic, 10,
        std::bind(&ManipulatorTorquePlugin::OnTorques, this, std::placeholders::_1)
      );

      // --- spin RCLCPP in its own thread ---
      std::thread([this]() { rclcpp::spin(node_); }).detach();

      // --- hook our Update callback so that
      //     if torques arrived earlier, we can still apply each world tick ---
      update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ManipulatorTorquePlugin::OnUpdate, this));

      RCLCPP_INFO(node_->get_logger(),
        "ManipulatorTorquePlugin loaded, subscribing to '%s'", topic.c_str());
    }

  private:
    void OnTorques(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      // store the latest array
      torques_ = msg->data;
    }

    void OnUpdate()
    {
      // lazy initialization of joint list
      if (!inited_) {
        // list your joint names here, in the same order as your controller publishes:
        joint_names_ = {
          "joint1","joint2","joint3","joint4","joint5","joint6"
        };
        for (auto & name : joint_names_) {
          auto j = model_->GetJoint(name);
          if (!j) {
            RCLCPP_WARN(node_->get_logger(),
              "Could not find joint [%s] in model", name.c_str());
          }
          joints_.push_back(j);
        }
        inited_ = true;
      }

      // apply the torques if we have the right size
      if (torques_.size() == joints_.size()) {
        for (size_t i = 0; i < joints_.size(); ++i) {
          if (joints_[i]) {
            // index 0 is the only axis for a revolute joint
            joints_[i]->SetForce(0, torques_[i]);
          }
        }
      }
    }

    // members
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;

    // ROS 2
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

    // storage
    std::vector<double> torques_;
    std::vector<std::string> joint_names_;
    std::vector<physics::JointPtr> joints_;
    bool inited_;
  };

  GZ_REGISTER_MODEL_PLUGIN(ManipulatorTorquePlugin)
}
