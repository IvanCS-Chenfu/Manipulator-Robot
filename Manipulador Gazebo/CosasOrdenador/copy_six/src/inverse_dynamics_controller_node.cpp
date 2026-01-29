#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <Eigen/Dense>

#include <chrono>
using namespace std::chrono_literals;

class InverseDynamicsController : public rclcpp::Node {
public:
  InverseDynamicsController()
  : Node("inverse_dynamics_controller")
  {
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    sub_H_   = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/dynamics/H", qos, std::bind(&InverseDynamicsController::cbH, this, std::placeholders::_1));
    sub_b_   = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/dynamics/b", qos, std::bind(&InverseDynamicsController::cbb, this, std::placeholders::_1));
    sub_ddq_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/desired_accelerations", qos, std::bind(&InverseDynamicsController::cbddq, this, std::placeholders::_1));

    pub_tau_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/desired_torques", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&InverseDynamicsController::loop, this));

    RCLCPP_INFO(get_logger(), "InverseDynamicsController ready");
  }

private:
  void cbH(const std_msgs::msg::Float64MultiArray::SharedPtr m){
    int N = std::lround(std::sqrt((double)m->data.size()));
    H_.resize(N, N);
    memcpy(H_.data(), m->data.data(), sizeof(double)*m->data.size());
    // lazily initialize b_ and ddq_ too:
    if (b_.size() != N) b_.setZero(N);
    if (ddq_.size() != N) ddq_.setZero(N);
  }

  void cbb(const std_msgs::msg::Float64MultiArray::SharedPtr m){
    b_ = Eigen::Map<const Eigen::VectorXd>(m->data.data(), m->data.size());
  }
  void cbddq(const std_msgs::msg::Float64MultiArray::SharedPtr m){
    ddq_ = Eigen::Map<const Eigen::VectorXd>(m->data.data(), m->data.size());
  }
  void loop(){
    if (H_.size() == 0 || b_.size() == 0 || ddq_.size() != H_.cols())
    {
      RCLCPP_INFO(get_logger(),"Waiting for all inputs…");
      return;
    }
    Eigen::VectorXd tau = H_*ddq_ + b_;
    std_msgs::msg::Float64MultiArray out;
    out.data.assign(tau.data(), tau.data()+tau.size());
    pub_tau_->publish(out);
    RCLCPP_INFO(get_logger(),"τ = [% .2f % .2f % .2f % .2f % .2f % .2f]",
      tau[0],tau[1],tau[2],tau[3],tau[4],tau[5]);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_H_, sub_b_, sub_ddq_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_tau_;
  rclcpp::TimerBase::SharedPtr timer_;

  Eigen::MatrixXd H_;
  Eigen::VectorXd b_, ddq_;
};

int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<InverseDynamicsController>());
  rclcpp::shutdown();
  return 0;
}