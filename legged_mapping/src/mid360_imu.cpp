#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "imu_complementary_filter/complementary_filter.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Mid360ImuNode : public rclcpp::Node
{
public:
  Mid360ImuNode()
  : Node("mid360_imu"),
    has_prev_stamp_(false),
    has_orientation_estimate_(false),
    q0_(1.0),
    q1_(0.0),
    q2_(0.0),
    q3_(0.0)
  {
    const std::string input_topic =
      this->declare_parameter<std::string>("input_topic", "livox/imu");
    const std::string output_topic =
      this->declare_parameter<std::string>("output_topic", "mid360/imu");
    int queue_size = this->declare_parameter<int>("queue_size", 50);
    const double gain_acc = this->declare_parameter<double>("gain_acc", 0.01);
    const double bias_alpha = this->declare_parameter<double>("bias_alpha", 0.01);
    const bool do_bias_estimation =
      this->declare_parameter<bool>("do_bias_estimation", true);
    const bool do_adaptive_gain = this->declare_parameter<bool>("do_adaptive_gain", true);
    max_dt_sec_ = this->declare_parameter<double>("max_dt_sec", 0.5);
    min_acc_norm_ = this->declare_parameter<double>("min_acc_norm", 1e-6);
    double orientation_variance =
      this->declare_parameter<double>("orientation_variance", 1e-3);

    if (queue_size <= 0) {
      RCLCPP_WARN(this->get_logger(), "queue_size <= 0, fallback to 50.");
      queue_size = 50;
    }
    if (max_dt_sec_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "max_dt_sec <= 0, fallback to 0.5.");
      max_dt_sec_ = 0.5;
    }
    if (min_acc_norm_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "min_acc_norm <= 0, fallback to 1e-6.");
      min_acc_norm_ = 1e-6;
    }
    if (orientation_variance <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "orientation_variance <= 0, fallback to 1e-3.");
      orientation_variance = 1e-3;
    }

    if (!filter_.setGainAcc(gain_acc)) {
      RCLCPP_WARN(this->get_logger(), "Invalid gain_acc=%.6f, using internal default.", gain_acc);
    }
    if (!filter_.setBiasAlpha(bias_alpha)) {
      RCLCPP_WARN(
        this->get_logger(), "Invalid bias_alpha=%.6f, using internal default.", bias_alpha);
    }
    filter_.setDoBiasEstimation(do_bias_estimation);
    filter_.setDoAdaptiveGain(do_adaptive_gain);

    default_orientation_covariance_.fill(0.0);
    default_orientation_covariance_[0] = orientation_variance;
    default_orientation_covariance_[4] = orientation_variance;
    default_orientation_covariance_[8] = orientation_variance;

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_size)));
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_topic, qos, std::bind(&Mid360ImuNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "mid360_imu started. input=%s output=%s queue_size=%d gain_acc=%.4f bias_alpha=%.4f",
      input_topic.c_str(), output_topic.c_str(), queue_size, gain_acc, bias_alpha);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (!has_prev_stamp_) {
      prev_stamp_ = stamp;
      has_prev_stamp_ = true;
      publishWithOrientation(msg);
      return;
    }

    const double dt = (stamp - prev_stamp_).seconds();
    prev_stamp_ = stamp;

    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;
    const double wx = msg->angular_velocity.x;
    const double wy = msg->angular_velocity.y;
    const double wz = msg->angular_velocity.z;

    bool do_update = true;
    if (dt <= 0.0 || dt > max_dt_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skip filter update due to invalid dt=%.6f s (range: (0, %.3f]).", dt, max_dt_sec_);
      do_update = false;
    }

    const double acc_norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (acc_norm < min_acc_norm_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Acceleration norm too small (%.9f), skip filter update.", acc_norm);
      do_update = false;
    }

    if (do_update) {
      filter_.update(ax, ay, az, wx, wy, wz, dt);
      filter_.getOrientation(q0_, q1_, q2_, q3_);
      has_orientation_estimate_ = true;
    }

    publishWithOrientation(msg);
  }

  void publishWithOrientation(const sensor_msgs::msg::Imu::SharedPtr & msg) const
  {
    sensor_msgs::msg::Imu out = *msg;
    if (has_orientation_estimate_) {
      out.orientation.w = q0_;
      out.orientation.x = q1_;
      out.orientation.y = q2_;
      out.orientation.z = q3_;
      if (out.orientation_covariance[0] < 0.0) {
        for (size_t i = 0; i < out.orientation_covariance.size(); ++i) {
          out.orientation_covariance[i] = default_orientation_covariance_[i];
        }
      }
    }
    pub_->publish(out);
  }

  imu_tools::ComplementaryFilter filter_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  rclcpp::Time prev_stamp_;
  bool has_prev_stamp_;
  bool has_orientation_estimate_;
  double q0_;
  double q1_;
  double q2_;
  double q3_;
  double max_dt_sec_;
  double min_acc_norm_;
  std::array<double, 9> default_orientation_covariance_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360ImuNode>());
  rclcpp::shutdown();
  return 0;
}
