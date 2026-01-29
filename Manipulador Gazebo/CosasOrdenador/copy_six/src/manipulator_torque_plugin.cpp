#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "gazebo_msgs/srv/apply_joint_effort.hpp"
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;
using ApplyEffort = gazebo_msgs::srv::ApplyJointEffort;

class TorqueApplier : public rclcpp::Node {
public:
  TorqueApplier()
  : Node("torque_applier")
  {
    client_ = create_client<gazebo_msgs::srv::ApplyJointEffort>("/gazebo/apply_joint_effort");
    while (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Waiting for /gazebo/apply_joint_effort …");
    }
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/desired_torques", qos,
      std::bind(&TorqueApplier::cb, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "TorqueApplier ready");
  }

private:
  void cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    static const std::vector<std::string> joints = {
      "joint1","joint2","joint3","joint4","joint5","joint6"
    };
    if (msg->data.size()!=joints.size()) return;
    rclcpp::Time now = this->now();
    for (size_t i=0;i<joints.size();++i) {
      auto req = std::make_shared<gazebo_msgs::srv::ApplyJointEffort::Request>();
      req->joint_name = joints[i];
      req->effort     = msg->data[i];
      req->start_time = now;
      req->duration.sec     = 0;
      req->duration.nanosec = 100000000;  // 0.1s
      client_->async_send_request(req);
    }
  }

  rclcpp::Client<gazebo_msgs::srv::ApplyJointEffort>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueApplier>());
  rclcpp::shutdown();
  return 0;
}