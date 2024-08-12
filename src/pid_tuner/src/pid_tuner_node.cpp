#include "pid_tuner_node.hpp"
#include <gs_controller_msgs/msg/gs_controller.hpp>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

PID_Tuner::PID_Tuner(
    const rclcpp::NodeOptions &options) : PID_Tuner("", options) {}

PID_Tuner::PID_Tuner(
    const std::string &name_space,
    const rclcpp::NodeOptions &options) : Node("PID_Tuner", name_space, options)
{

    this->declare_parameter("max_data_points", 1000); // デフォルト値を100に設定
    this->get_parameter("max_data_points", max_data_points_);

    this->declare_parameter("time_const", 5.0); // デフォルト値を0.5に設定
    this->get_parameter("time_const", time_constant_);

    this->declare_parameter("diff_time", 0.1); // 数値積分サンプリング時間
    this->get_parameter("diff_time", diff_time_);

    this->declare_parameter("mode_selector", 0); // 制御器の切り替え
    this->get_parameter("mode_selector", mode_selector_);

    subscription_ = this->create_subscription<gs_controller_msgs::msg::GsController>("plant_info", 10, std::bind(&PID_Tuner::tuning_callback, this, std::placeholders::_1));
    test_publisher_ = this->create_publisher<gs_controller_msgs::msg::GsController>("Compute_Td", 1); // Td out
}
void PID_Tuner::tuning_callback(const gs_controller_msgs::msg::GsController::SharedPtr msg)
{
    x_data_.push_back(msg->output-compute_ve(msg->output)); // 受信したデータをx_data_に保存する処理
    y_data_.push_back(compute_vu(msg->input));                // 受信したデータをy_data_に保存する処理
    v_data_.push_back(msg->gs_parameter);                // 受信したデータをv_data_に保存する処理

    // バッファがたまったら最小二乗法によるゲイン計算を実施
    if (x_data_.size() > max_data_points_ && !is_fit_calculated_)
    {
        static float pre_integral_ = 0.0;

        switch (mode_selector_)
        {
        case 0:
        {
            Eigen::MatrixXd A(max_data_points_, 6);
            Eigen::VectorXd b(max_data_points_);
            Eigen::MatrixXd kp_temp(max_data_points_, 3);
            Eigen::MatrixXd ki_temp(max_data_points_, 3);
            double x_intg;
            
            for (size_t i = 0; i < max_data_points_; i++) {
                x_intg=pre_integral_ + x_data_[i] * diff_time_;
                kp_temp.row(i) << x_data_[i], x_data_[i] * v_data_[i], x_data_[i] * v_data_[i] * v_data_[i];
                ki_temp.row(i) << x_intg, x_intg * v_data_[i], x_intg * v_data_[i] * v_data_[i];
                b(i) = y_data_[i];
                pre_integral_ = x_intg;
            }
            A << kp_temp, ki_temp;
            // Aの転置行列を計算
            Eigen::MatrixXd A_transpose = A.transpose();
            // Aの転置とAの積から疑似逆行列を計算（A^T * A）^(-1) * A^T
            Eigen::MatrixXd pseudo_inverse = (A_transpose * A).completeOrthogonalDecomposition().pseudoInverse() * A_transpose;
            // 疑似逆行列を使用して最小二乗解を求める
            Eigen::VectorXd params = pseudo_inverse * b;
            RCLCPP_INFO(this->get_logger(), "Fitted parameters: kp=%f+%f*x+%f*x^2, ki=%f+%f*x+%f*x^2", params[0], params[1], params[2], params[3], params[4], params[5]); 
        }
        break;

        case 1:
        {
            Eigen::MatrixXd Ap(max_data_points_, 2);
            Eigen::VectorXd bp(max_data_points_);

            for (size_t i = 0; i < max_data_points_; i++)
            {
                Ap(i, 0) = x_data_[i];                              // 比例制御の疑似制御入力計算
                Ap(i, 1) = pre_integral_ + x_data_[i] * diff_time_; // 積分制御の疑似制御入力計算

                bp(i) = y_data_[i];
                pre_integral_ = Ap(i, 1);
            }
            // Aの転置行列を計算
            Eigen::MatrixXd A_transpose_p = Ap.transpose();
            // Aの転置とAの積から疑似逆行列を計算（A^T * A）^(-1) * A^T
            Eigen::MatrixXd pseudo_inverse_p = (A_transpose_p * Ap).completeOrthogonalDecomposition().pseudoInverse() * A_transpose_p;
            // 疑似逆行列を使用して最小二乗解を求める
            Eigen::VectorXd params_p = pseudo_inverse_p * bp;
            RCLCPP_INFO(this->get_logger(), "Fitted parameters: kp=%f, ki=%f", params_p[0], params_p[1]); // PID
        }
            break;
        }

        // 以降の計算フラグを無効化
        is_fit_calculated_ = true;
    }
}
double PID_Tuner::compute_ve(double u)
{
    static double x_1; // 状態変数
    double ve;          // 　出力変数
    // dx(t)/dt=Ax(t)+Bu(t)に対する4次ルンゲクッタ
    // x_1とuの時刻は同じ
    k1 = (-(1.0 / time_constant_) * x_1 + (1.0 / time_constant_) * u);
    k2 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k1 / 2.0) + (1.0 / time_constant_) * u);
    k3 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k2 / 2.0) + (1.0 / time_constant_) * u);
    k4 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k3) + (1.0 / time_constant_) * u);

    ve = x_1;                                                        // フィルタ出力計算
    x_1 = x_1 + diff_time_ * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0; // 1サンプル先の状態変数を更新

    return ve;
}
double PID_Tuner::compute_vu(double u)
{
    static double x_2; // 状態変数
    double vu;          // 　出力変数
    // dx(t)/dt=Ax(t)+Bu(t)に対する4次ルンゲクッタ
    // x_1とuの時刻は同じ
    k1 = (-(1.0 / time_constant_) * x_2 + (1.0 / time_constant_) * u);
    k2 = (-(1.0 / time_constant_) * (x_2 + diff_time_ * k1 / 2.0) + (1.0 / time_constant_) * u);
    k3 = (-(1.0 / time_constant_) * (x_2 + diff_time_ * k2 / 2.0) + (1.0 / time_constant_) * u);
    k4 = (-(1.0 / time_constant_) * (x_2 + diff_time_ * k3) + (1.0 / time_constant_) * u);

    vu = x_2;                                                        // フィルタ出力計算
    x_2 = x_2 + diff_time_ * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0; // 1サンプル先の状態変数を更新

    return vu;
}