#ifndef PID_TUNER_HPP_
#define PID_TUNER_HPP_

#include <deque>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <gs_controller_msgs/msg/gs_controller.hpp>

class PID_Tuner : public rclcpp::Node
{
public:
  PID_Tuner(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  PID_Tuner(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );


private:
    void tuning_callback(const gs_controller_msgs::msg::GsController::SharedPtr msg);
    double compute_ve(double u);//参照モデルTd1の計算処理
    double compute_vu(double u);//参照モデルTd2の計算処理

    std::deque<double> x_data_;
    std::deque<double> y_data_;
    double integral_;//積分値
    size_t max_data_points_;//最大データ点数
    bool is_fit_calculated_;
    double time_constant_;//参照モデルの時定数
    int mode_selector_;

    rclcpp::Subscription<gs_controller_msgs::msg::GsController>::SharedPtr subscription_;
    rclcpp::Publisher<gs_controller_msgs::msg::GsController>::SharedPtr test_publisher_;

    double diff_time_;//差分時間
    double k1;//ルンゲクッタ中間変数1
    double k2;//ルンゲクッタ中間変数2
    double k3;//ルンゲクッタ中間変数3
    double k4;//ルンゲクッタ中間変数4
};
#endif  // PID_TUNER_HPP_