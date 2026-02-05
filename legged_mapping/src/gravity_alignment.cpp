/**
 * @file gravity_alignment.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief Estimate gravity-aligned frame from IMU acceleration average.
 * @version 0.1
 * @date 2026-02-05
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace legged
{

class GravityAlignmentNode : public rclcpp::Node
{
public:
  GravityAlignmentNode()
  : Node("gravity_alignment_node"),
    initialized_(false),
    valid_sample_count_(0),
    mean_acc_(Eigen::Vector3d::Zero())
  {
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "livox/imu");
    source_frame_id_ = this->declare_parameter<std::string>("source_frame_id", "camera_init");
    aligned_frame_id_ =
      this->declare_parameter<std::string>("aligned_frame_id", "camera_init_aligned");
    sample_count_ = this->declare_parameter<int>("sample_count", 200);
    queue_size_ = this->declare_parameter<int>("queue_size", 50);
    min_acc_norm_ = this->declare_parameter<double>("min_acc_norm", 1e-6);

    if (sample_count_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "sample_count <= 0, fallback to 1.");
      sample_count_ = 1;
    }
    if (queue_size_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "queue_size <= 0, fallback to 50.");
      queue_size_ = 50;
    }
    if (min_acc_norm_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "min_acc_norm <= 0, fallback to 1e-6.");
      min_acc_norm_ = 1e-6;
    }
    if (source_frame_id_.empty()) {
      RCLCPP_WARN(this->get_logger(), "source_frame_id is empty, fallback to camera_init.");
      source_frame_id_ = "camera_init";
    }
    if (aligned_frame_id_.empty()) {
      RCLCPP_WARN(
        this->get_logger(), "aligned_frame_id is empty, fallback to camera_init_aligned.");
      aligned_frame_id_ = "camera_init_aligned";
    }
    if (aligned_frame_id_ == source_frame_id_) {
      RCLCPP_WARN(
        this->get_logger(),
        "aligned_frame_id equals source_frame_id (%s). TF remains valid but hard to inspect.",
        aligned_frame_id_.c_str());
    }

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_size_)));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos, std::bind(&GravityAlignmentNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "gravity_alignment_node started. imu_topic=%s source_frame_id=%s aligned_frame_id=%s "
      "sample_count=%d",
      imu_topic_.c_str(), source_frame_id_.c_str(), aligned_frame_id_.c_str(), sample_count_);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (initialized_) {
      return;
    }

    const Eigen::Vector3d acc(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    const double acc_norm = acc.norm();
    if (acc_norm < min_acc_norm_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skip invalid acceleration norm %.9f (threshold %.9f).", acc_norm, min_acc_norm_);
      return;
    }

    ++valid_sample_count_;
    mean_acc_ += (acc - mean_acc_) / static_cast<double>(valid_sample_count_);

    if (valid_sample_count_ < static_cast<size_t>(sample_count_)) {
      return;
    }

    publishStaticTransform();
  }

  void publishStaticTransform()
  {
    const double mean_norm = mean_acc_.norm();
    if (mean_norm < min_acc_norm_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Mean acceleration norm %.9f is invalid, cannot compute gravity alignment.", mean_norm);
      return;
    }

    Eigen::Quaterniond q_aligned_to_source = Eigen::Quaterniond::FromTwoVectors(
      mean_acc_, Eigen::Vector3d::UnitZ());
    q_aligned_to_source.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = aligned_frame_id_;
    tf_msg.child_frame_id = source_frame_id_;
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.w = q_aligned_to_source.w();
    tf_msg.transform.rotation.x = q_aligned_to_source.x();
    tf_msg.transform.rotation.y = q_aligned_to_source.y();
    tf_msg.transform.rotation.z = q_aligned_to_source.z();

    static_broadcaster_->sendTransform(tf_msg);
    initialized_ = true;
    imu_sub_.reset();

    RCLCPP_INFO(
      this->get_logger(),
      "Published static TF %s -> %s after %zu samples. mean_acc=[%.6f %.6f %.6f], "
      "q_aligned_to_source=[w=%.6f x=%.6f y=%.6f z=%.6f]",
      aligned_frame_id_.c_str(), source_frame_id_.c_str(), valid_sample_count_,
      mean_acc_.x(), mean_acc_.y(), mean_acc_.z(),
      q_aligned_to_source.w(), q_aligned_to_source.x(),
      q_aligned_to_source.y(), q_aligned_to_source.z());
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::string imu_topic_;
  std::string source_frame_id_;
  std::string aligned_frame_id_;
  int sample_count_;
  int queue_size_;
  double min_acc_norm_;

  bool initialized_;
  size_t valid_sample_count_;
  Eigen::Vector3d mean_acc_;
};

}  // namespace legged

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<legged::GravityAlignmentNode>());
  rclcpp::shutdown();
  return 0;
}
