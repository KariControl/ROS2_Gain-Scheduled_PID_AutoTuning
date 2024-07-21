#ifndef DATA_SYNCHRONIZER_HPP
#define DATA_SYNCHRONIZER_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include <gs_controller_msgs/msg/gs_controller.hpp>

class DataSynchronizer : public rclcpp::Node {
public:
    DataSynchronizer();

private:
    void callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& vel_msg,const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& str_msg ,const sensor_msgs::msg::Imu::ConstSharedPtr& yaw_rate_msg) ;
    
    rclcpp::Publisher<gs_controller_msgs::msg::GsController>::SharedPtr plant_publisher_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> yaw_rate_sub_;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> vel_sub_;
    message_filters::Subscriber<ackermann_msgs::msg::AckermannDriveStamped> input_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped,ackermann_msgs::msg::AckermannDriveStamped, sensor_msgs::msg::Imu>>> sync_;
};

#endif  // DATA_SYNCHRONIZER_HPP
