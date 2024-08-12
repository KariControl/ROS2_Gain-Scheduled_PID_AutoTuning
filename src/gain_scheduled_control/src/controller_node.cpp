#include "control_sim/controller_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

ControllerNode::ControllerNode(
  const rclcpp::NodeOptions& options
): ControllerNode("",options){}

ControllerNode::ControllerNode(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("ControllerNode", name_space, options){
    this->declare_parameter("set_point", 0.1);  // デフォルト値として1.0を設定
    this->declare_parameter("kp", 0.2);         // デフォルト値として0.1を設定
    this->declare_parameter("ki", 0.3);        // デフォルト値として0.01を設定
    this->declare_parameter("dt", 0.01);        // デフォルト値として0.05を設定

    this->declare_parameter("wP0", 0.0);        // 比例ゲインスケジュール則ハイパーパラメータ1
    this->declare_parameter("wP1", 0.0);        // 比例ゲインスケジュール則ハイパーパラメータ2
    this->declare_parameter("wP2", 0.01);        // 比例ゲインスケジュール則ハイパーパラメータ3

    this->declare_parameter("wI0", 0.0);        // 積分ゲインスケジュール則ハイパーパラメータ1
    this->declare_parameter("wI1", 0.0);        // 積分ゲインスケジュール則ハイパーパラメータ2
    this->declare_parameter("wI2", 0.0);        // 積分ゲインスケジュール則ハイパーパラメータ3

    this->get_parameter("set_point",setpoint_);
    this->get_parameter("kp",kp_);
    this->get_parameter("ki",ki_);
    this->get_parameter("dt",dt_);

    this->get_parameter("wP0",wP0_);
    this->get_parameter("wP1",wP1_);
    this->get_parameter("wP2",wP2_);

    this->get_parameter("wI0",wI0_);
    this->get_parameter("wI1",wI1_);
    this->get_parameter("wI2",wI2_);

    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("vehicle_velocity", 1, std::bind(&ControllerNode::Gain_scheduled_callback, this, std::placeholders::_1));
    target_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/MotionPlanning", 1, std::bind(&ControllerNode::reference_callback, this, std::placeholders::_1));
    yaw_rate_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("vehicle_state", 1, std::bind(&ControllerNode::yaw_rate_callback, this, std::placeholders::_1));

    controller_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("steering", 1);

    using namespace std::literals::chrono_literals; 
    timer_ = this->create_wall_timer(10ms, std::bind(&ControllerNode::timer_callback, this));
}
void ControllerNode::Gain_scheduled_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    velocity_ = msg->twist.linear.x;
    kp_=wP0_+wP1_*velocity_+wP2_*velocity_*velocity_;
    ki_=wI0_+wI1_*velocity_+wI2_*velocity_*velocity_;
}
void ControllerNode::yaw_rate_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    yaw_rate_ = msg->angular_velocity.z;
}
void ControllerNode::reference_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    setpoint_ = msg->twist.angular.z;
}
void ControllerNode::timer_callback() {
    double error;
    static double pre_integral_ = 0.0;
    double output; 

    error=setpoint_ - yaw_rate_;
    integral_= pre_integral_+error * dt_;
    output= kp_ * error + ki_ * integral_;
    pre_integral_ = integral_;

    ackermann_msgs::msg::AckermannDriveStamped output_msg;
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = "base_link";
    output_msg.drive.steering_angle = output;
    controller_publisher_->publish(output_msg);
}