#include "time_sync.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <gs_controller_msgs/msg/gs_controller.hpp>

using namespace message_filters;

DataSynchronizer::DataSynchronizer() : Node("data_synchronizer"),
    vel_sub_(this, "vehicle_velocity"),
    input_sub_(this, "steering"),
    yaw_rate_sub_(this, "vehicle_state")
{
    typedef sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped,ackermann_msgs::msg::AckermannDriveStamped, sensor_msgs::msg::Imu> MySyncPolicy;
    sync_ = std::make_shared<Synchronizer<MySyncPolicy>>(MySyncPolicy(10), vel_sub_, input_sub_,yaw_rate_sub_);
    sync_->registerCallback(std::bind(&DataSynchronizer::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    plant_publisher_ = this->create_publisher<gs_controller_msgs::msg::GsController>("plant_info", 1);// controller mesage
}
void DataSynchronizer::callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& vel_msg,const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& str_msg,const sensor_msgs::msg::Imu::ConstSharedPtr& yaw_rate_msg) {
    RCLCPP_INFO(this->get_logger(), "同期されたメッセージを受信しました。");
    RCLCPP_INFO(this->get_logger(), "スケジューリングパラメータ: %f m/s", vel_msg->twist.linear.x);
    RCLCPP_INFO(this->get_logger(), "出力: %f rad/s", yaw_rate_msg->angular_velocity.z);
    RCLCPP_INFO(this->get_logger(), "入力: %f m/s", str_msg->drive.steering_angle);

    gs_controller_msgs::msg::GsController output_msg;
    output_msg.header.stamp = vel_msg->header.stamp;
    output_msg.header.frame_id = "base_link";
    output_msg.output = yaw_rate_msg->angular_velocity.x; 
    output_msg.input = str_msg->drive.steering_angle;
    output_msg.gs_parameter = vel_msg->twist.linear.x; 
    plant_publisher_->publish(output_msg);
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataSynchronizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}