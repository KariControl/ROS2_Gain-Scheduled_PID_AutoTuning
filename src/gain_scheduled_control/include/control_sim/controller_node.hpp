#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  ControllerNode(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );

private:
  void timer_callback();
  void Gain_scheduled_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void yaw_rate_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void reference_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controller_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr yaw_rate_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double steering_angle_;
  double velocity_;
  double yaw_rate_;
  double kp_;
  double ki_;
  double dt_;
  double setpoint_;
  double integral_;
  double wP0_;
  double wP1_;
  double wP2_;
  double wI0_;
  double wI1_;
  double wI2_;
};