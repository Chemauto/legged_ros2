/**
 * @file dual_imu_static_tf.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief Use base/lidar IMU acceleration averages to initialize and publish static TF:
 *        odom -> camera_init and lidar(body) -> base.
 * @version 0.1
 * @date 2026-02-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "unitree_go/msg/low_state.hpp"

namespace legged
{

class DualImuStaticTfNode : public rclcpp::Node
{
public:
  DualImuStaticTfNode()
  : Node("dual_imu_static_tf_node"),
    initialized_(false),
    base_sample_count_(0),
    lidar_sample_count_(0),
    base_mean_acc_(Eigen::Vector3d::Zero()),
    lidar_mean_acc_(Eigen::Vector3d::Zero())
  {
    base_imu_topic_ = this->declare_parameter<std::string>("base_imu_topic", "lowstate");
    lidar_imu_topic_ = this->declare_parameter<std::string>("lidar_imu_topic", "/livox/imu");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    camera_init_frame_ = this->declare_parameter<std::string>("camera_init_frame", "camera_init");
    lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "body");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base");
    init_window_ = this->declare_parameter<int>("init_window", 200);
    queue_size_ = this->declare_parameter<int>("queue_size", 50);
    min_acc_norm_ = this->declare_parameter<double>("min_acc_norm", 1e-6);

    const auto t_base_to_lidar_param =
      this->declare_parameter<std::vector<double>>("t_base_to_lidar", {0.1870, 0.0, 0.0803});
    t_base_to_lidar_ = parseTranslation(t_base_to_lidar_param);

    if (init_window_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "init_window <= 0, fallback to 1.");
      init_window_ = 1;
    }
    if (queue_size_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "queue_size <= 0, fallback to 50.");
      queue_size_ = 50;
    }
    if (min_acc_norm_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "min_acc_norm <= 0.0, fallback to 1e-6.");
      min_acc_norm_ = 1e-6;
    }
    if (odom_frame_.empty()) {
      RCLCPP_WARN(this->get_logger(), "odom_frame is empty, fallback to odom.");
      odom_frame_ = "odom";
    }
    if (camera_init_frame_.empty()) {
      RCLCPP_WARN(this->get_logger(), "camera_init_frame is empty, fallback to camera_init.");
      camera_init_frame_ = "camera_init";
    }
    if (lidar_frame_.empty()) {
      RCLCPP_WARN(this->get_logger(), "lidar_frame is empty, fallback to body.");
      lidar_frame_ = "body";
    }
    if (base_frame_.empty()) {
      RCLCPP_WARN(this->get_logger(), "base_frame is empty, fallback to base.");
      base_frame_ = "base";
    }

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_size_)));
    base_imu_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      base_imu_topic_, qos,
      std::bind(&DualImuStaticTfNode::baseImuCallback, this, std::placeholders::_1));
    lidar_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      lidar_imu_topic_, qos,
      std::bind(&DualImuStaticTfNode::lidarImuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "dual_imu_static_tf_node started. base_imu_topic=%s lidar_imu_topic=%s "
      "odom_frame=%s camera_init_frame=%s lidar_frame=%s base_frame=%s init_window=%d",
      base_imu_topic_.c_str(), lidar_imu_topic_.c_str(),
      odom_frame_.c_str(), camera_init_frame_.c_str(),
      lidar_frame_.c_str(), base_frame_.c_str(), init_window_);
  }

private:
  static double extractPitchRad(const Eigen::Quaterniond & q)
  {
    // roll-pitch-yaw extraction with pitch around +Y.
    const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    return std::asin(std::clamp(sinp, -1.0, 1.0));
  }

  Eigen::Vector3d parseTranslation(const std::vector<double> & raw) const
  {
    if (raw.size() == 3) {
      return Eigen::Vector3d(raw[0], raw[1], raw[2]);
    }
    RCLCPP_WARN(
      this->get_logger(), "t_base_to_lidar must be [x, y, z], using [0.1870, 0.0, 0.0803].");
    return Eigen::Vector3d(0.1870, 0.0, 0.0803);
  }

  bool validateAcceleration(
    const Eigen::Vector3d & a, const char * label,
    const rclcpp::Time & stamp)
  {
    const double norm = a.norm();
    if (norm >= min_acc_norm_) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[%s] Skip invalid acceleration norm %.9f (threshold %.9f), stamp=%.6f.",
      label, norm, min_acc_norm_, stamp.seconds());
    return false;
  }

  void updateRunningMean(const Eigen::Vector3d & sample, size_t & count, Eigen::Vector3d & mean)
  {
    if (count >= static_cast<size_t>(init_window_)) {
      return;
    }
    ++count;
    mean += (sample - mean) / static_cast<double>(count);
  }

  void baseImuCallback(const unitree_go::msg::LowState::SharedPtr msg)
  {
    if (initialized_) {
      return;
    }

    const Eigen::Vector3d acc(
      msg->imu_state.accelerometer[0],
      msg->imu_state.accelerometer[1],
      msg->imu_state.accelerometer[2]);
    const rclcpp::Time stamp = this->now();
    if (!validateAcceleration(acc, "base", stamp)) {
      return;
    }

    updateRunningMean(acc, base_sample_count_, base_mean_acc_);
    tryInitialize();
  }

  void lidarImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (initialized_) {
      return;
    }

    const Eigen::Vector3d acc(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    const rclcpp::Time stamp(msg->header.stamp);
    if (!validateAcceleration(acc, "lidar", stamp)) {
      return;
    }

    updateRunningMean(acc, lidar_sample_count_, lidar_mean_acc_);
    tryInitialize();
  }

  void tryInitialize()
  {
    if (initialized_) {
      return;
    }
    if (
      base_sample_count_ < static_cast<size_t>(init_window_) ||
      lidar_sample_count_ < static_cast<size_t>(init_window_))
    {
      return;
    }

    // Convention: q_src_to_des * v_des = v_src.
    const Eigen::Quaterniond q_level_to_base =
      Eigen::Quaterniond::FromTwoVectors(base_mean_acc_, Eigen::Vector3d::UnitZ());
    const Eigen::Quaterniond q_level_to_lidar =
      Eigen::Quaterniond::FromTwoVectors(lidar_mean_acc_, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q_lidar_to_base_raw =
      q_level_to_lidar.conjugate() * q_level_to_base;
    q_lidar_to_base_raw.normalize();

    const double pitch_raw = extractPitchRad(q_lidar_to_base_raw);
    Eigen::Quaterniond q_lidar_to_base(Eigen::AngleAxisd(pitch_raw, Eigen::Vector3d::UnitY()));
    q_lidar_to_base.normalize();

    const Eigen::Vector3d t_lidar_to_base = -(q_lidar_to_base * t_base_to_lidar_);
    const Eigen::Vector3d t_odom_to_camera_init = q_level_to_base * t_base_to_lidar_;
    const Eigen::Quaterniond q_odom_to_camera_init = q_level_to_lidar;

    const auto now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped tf_odom_to_camera_init;
    tf_odom_to_camera_init.header.stamp = now;
    tf_odom_to_camera_init.header.frame_id = odom_frame_;
    tf_odom_to_camera_init.child_frame_id = camera_init_frame_;
    tf_odom_to_camera_init.transform.translation.x = t_odom_to_camera_init.x();
    tf_odom_to_camera_init.transform.translation.y = t_odom_to_camera_init.y();
    tf_odom_to_camera_init.transform.translation.z = t_odom_to_camera_init.z();
    tf_odom_to_camera_init.transform.rotation.w = q_odom_to_camera_init.w();
    tf_odom_to_camera_init.transform.rotation.x = q_odom_to_camera_init.x();
    tf_odom_to_camera_init.transform.rotation.y = q_odom_to_camera_init.y();
    tf_odom_to_camera_init.transform.rotation.z = q_odom_to_camera_init.z();

    geometry_msgs::msg::TransformStamped tf_lidar_to_base;
    tf_lidar_to_base.header.stamp = now;
    tf_lidar_to_base.header.frame_id = lidar_frame_;
    tf_lidar_to_base.child_frame_id = base_frame_;
    tf_lidar_to_base.transform.translation.x = t_lidar_to_base.x();
    tf_lidar_to_base.transform.translation.y = t_lidar_to_base.y();
    tf_lidar_to_base.transform.translation.z = t_lidar_to_base.z();
    tf_lidar_to_base.transform.rotation.w = q_lidar_to_base.w();
    tf_lidar_to_base.transform.rotation.x = q_lidar_to_base.x();
    tf_lidar_to_base.transform.rotation.y = q_lidar_to_base.y();
    tf_lidar_to_base.transform.rotation.z = q_lidar_to_base.z();

    static_broadcaster_->sendTransform({tf_odom_to_camera_init, tf_lidar_to_base});
    initialized_ = true;
    base_imu_sub_.reset();
    lidar_imu_sub_.reset();

    constexpr double kRadToDeg = 57.295779513082320876;
    RCLCPP_INFO(
      this->get_logger(),
      "Published static TFs after %zu/%zu samples. "
      "base_mean_acc=[%.6f %.6f %.6f], lidar_mean_acc=[%.6f %.6f %.6f], "
      "pitch(lidar->base)=%.6f deg, "
      "q_odom_to_camera_init=[w=%.6f x=%.6f y=%.6f z=%.6f], "
      "t_odom_to_camera_init=[%.6f %.6f %.6f], "
      "t_lidar_to_base=[%.6f %.6f %.6f]",
      base_sample_count_, lidar_sample_count_,
      base_mean_acc_.x(), base_mean_acc_.y(), base_mean_acc_.z(),
      lidar_mean_acc_.x(), lidar_mean_acc_.y(), lidar_mean_acc_.z(),
      pitch_raw * kRadToDeg,
      q_odom_to_camera_init.w(), q_odom_to_camera_init.x(),
      q_odom_to_camera_init.y(), q_odom_to_camera_init.z(),
      t_odom_to_camera_init.x(),
      t_odom_to_camera_init.y(),
      t_odom_to_camera_init.z(),
      t_lidar_to_base.x(), t_lidar_to_base.y(), t_lidar_to_base.z());
  }

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr base_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lidar_imu_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::string base_imu_topic_;
  std::string lidar_imu_topic_;
  std::string odom_frame_;
  std::string camera_init_frame_;
  std::string lidar_frame_;
  std::string base_frame_;
  int init_window_;
  int queue_size_;
  double min_acc_norm_;
  Eigen::Vector3d t_base_to_lidar_;

  bool initialized_;
  size_t base_sample_count_;
  size_t lidar_sample_count_;
  Eigen::Vector3d base_mean_acc_;
  Eigen::Vector3d lidar_mean_acc_;
};

}  // namespace legged

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<legged::DualImuStaticTfNode>());
  rclcpp::shutdown();
  return 0;
}
