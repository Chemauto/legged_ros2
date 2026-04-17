/**
 * @file legged_ros2_control.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "hardware_interface/component_parser.hpp"
#include "realtime_tools/realtime_helpers.hpp"

#include "legged_ros2_control/legged_ros2_control.hpp"


namespace legged
{
LeggedRos2Control::LeggedRos2Control(rclcpp::Node::SharedPtr node) : 
  node_(node), 
  logger_(rclcpp::get_logger(node_->get_name()+std::string(".legged_ros2_control"))) // TODO: change logger name
{
}

LeggedRos2Control::~LeggedRos2Control()
{
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->remove_node(node_);  // 同时移除 node_
  cm_executor_->cancel();

  if (cm_thread_.joinable()) cm_thread_.join();
  if (spin_thread_.joinable()) spin_thread_.join();
}

std::string LeggedRos2Control::get_robot_description_()
{
  // Getting robot description from parameter first. If not set trying from topic
  std::string robot_description;

  auto node = std::make_shared<rclcpp::Node>(
    "robot_description_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (node->has_parameter("robot_description"))
  {
    robot_description = node->get_parameter("robot_description").as_string();
    return robot_description;
  }

  RCLCPP_WARN(
    logger_,
    "Failed to get robot_description from parameter. Will listen on the ~/robot_description "
    "topic...");

  auto robot_description_sub = node->create_subscription<std_msgs::msg::String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    [&](const std_msgs::msg::String::SharedPtr msg)
    {
      if (!msg->data.empty() && robot_description.empty()) robot_description = msg->data;
    });

  while (robot_description.empty() && rclcpp::ok())
  {
    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  return robot_description;
}


void LeggedRos2Control::init()
{

  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // ------------------------------------------------------------------
  // Get the robot description from parameter or topic
  // ------------------------------------------------------------------

  urdf_string_ = this->get_robot_description_();

  // ------------------------------------------------------------------
  // Prepeare the controller manager
  // ------------------------------------------------------------------

  RCLCPP_INFO(logger_, "Creating controller manager...");

  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";
  // <!-- #########jazzy########## -->
  // 当前分支旧代码:
  // auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string_);
  // auto resource_manager = std::make_unique<hardware_interface::ResourceManager>();
  // resource_manager->load_urdf(urdf_string_, false, false);
  // import_components_(hardware_info, resource_manager);
  // controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
  //   std::move(resource_manager), cm_executor_, manager_node_name, node_->get_namespace());
  // <!-- #########jazzy########## -->
  // <!-- #########new########## -->
  // 新代码,使用jazzy版本
  auto controller_manager_options = controller_manager::get_cm_node_options();
  controller_manager_options.arguments(
    {"--ros-args", "-r", "/robot_description:=/controller_manager/ignored_robot_description"});
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    cm_executor_, urdf_string_, true, manager_node_name, node_->get_namespace(),
    controller_manager_options);
  // <!-- #########new########## -->
  cm_executor_->add_node(controller_manager_);
  cm_executor_->add_node(node_);

  RCLCPP_INFO(logger_, "Controller manager successfully created.");

  // ------------------------------------------------------------------
  // Set the update loop for the controller manager
  // ------------------------------------------------------------------

  const bool use_sim_time = controller_manager_->get_parameter_or<bool>("use_sim_time", false);
  RCLCPP_INFO(
    logger_, "Controller manager using simulation time: %s", use_sim_time ? "true" : "false");

  const int cpu_affinity = controller_manager_->get_parameter_or<int>("cpu_affinity", -1);
  if(cpu_affinity>=0){
    const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity);
    if(!affinity_result.first){
      RCLCPP_WARN(logger_, "Unable to set the CPU affinity : '%s'", affinity_result.second.c_str());
    }
  }

  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory = controller_manager_->get_parameter_or<bool>("lock_memory", has_realtime);
  if(lock_memory){
    const auto lock_result = realtime_tools::lock_memory();
    if(!lock_result.first){
      RCLCPP_WARN(logger_, "Unable to lock memory : '%s'", lock_result.second.c_str());
    }
  }

  if(!controller_manager_->has_parameter("update_rate")){
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }
  update_rate_ = controller_manager_->get_parameter("update_rate").as_int();
  RCLCPP_INFO(logger_, "Controller manager update rate: %d Hz", update_rate_);

  const int thread_priority = controller_manager_->get_parameter_or<int>("thread_priority", 50);
  RCLCPP_INFO(
    logger_, "Spawning %s RT thread with scheduler priority: %d", controller_manager_->get_name(), thread_priority);

  cm_thread_ = std::thread(
    [&](){
      if(realtime_tools::has_realtime_kernel()){
        if (!realtime_tools::configure_sched_fifo(thread_priority)){
          RCLCPP_WARN(
            this->logger_,
            "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
            "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
            "for details on how to enable realtime scheduling.",
            errno, strerror(errno));
        } else {
          RCLCPP_INFO(
            this->logger_, "Successful set up FIFO RT scheduling policy with priority %i.",
            thread_priority);
        }
      } else {
        RCLCPP_WARN(
          this->logger_,
          "No real-time kernel detected on this system. See "
          "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
          "for details on how to enable realtime scheduling.");
      } // end if has_realtime

      const auto period = std::chrono::nanoseconds(1'000'000'000 / update_rate_);
      // <!-- #########jazzy########## -->
      // const auto cm_now = std::chrono::nanoseconds(this->controller_manager_->now().nanoseconds());
      // std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{cm_now};
      // rclcpp::Time previous_time = this->controller_manager_->now();
      // <!-- #########jazzy########## -->
      // <!-- #########new########## -->
      // 新代码,使用jazzy版本
      auto trigger_clock = this->controller_manager_->get_trigger_clock();
      auto next_iteration_time = std::chrono::steady_clock::now();
      rclcpp::Time previous_time = trigger_clock->now();
      // <!-- #########new########## -->
      rclcpp::Duration period_duration = rclcpp::Duration::from_nanoseconds(period.count());
      rclcpp::Duration period_error_threshold = rclcpp::Duration::from_nanoseconds(0.1 * period.count()); // 10% threshold

      while(rclcpp::ok()){
        // calculate measured period
        // <!-- #########jazzy########## -->
        // const auto current_time = this->controller_manager_->now();
        // <!-- #########jazzy########## -->
        // <!-- #########new########## -->
        // 新代码,使用jazzy版本
        const auto current_time = trigger_clock->now();
        // <!-- #########new########## -->
        const auto measured_period = current_time - previous_time;
        previous_time = current_time;

        // execute update loop
        this->update(current_time, measured_period);

        // wait until we hit the end of the period
        next_iteration_time += period;

        // if(measured_period - period_duration > period_error_threshold){
        //   RCLCPP_WARN_THROTTLE(
        //     logger_,
        //     *node_->get_clock(),
        //     1000, // 1 second throttle
        //     "Measured period (%f s) is larger than expected period (%f s). "
        //     "This can lead to performance issues.",
        //     measured_period.seconds(), period_duration.seconds());
        // }

        if(use_sim_time){ // TODO: check sim time
          // <!-- #########jazzy########## -->
          // this->controller_manager_->get_clock()->sleep_until(current_time + period);
          // <!-- #########jazzy########## -->
          // <!-- #########new########## -->
          // 新代码,使用jazzy版本
          trigger_clock->sleep_until(current_time + period);
          // <!-- #########new########## -->
        } else {
          std::this_thread::sleep_until(next_iteration_time);
        }
      }
    }
  );

  // sched_param sched;
  // sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
  // if (pthread_setschedparam(cm_thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
  //   RCLCPP_ERROR(logger_, "Failed to set thread scheduling policy to FIFO");
  // } else {
  //   RCLCPP_INFO(logger_, "Controller manager thread scheduling policy set to FIFO");
  // }

  spin_thread_ = std::thread(
    [this](){
      RCLCPP_INFO(logger_, "Spinning controller manager executor in a separate thread");
      this->cm_executor_->spin();
    }
  );
}


void LeggedRos2Control::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  this->controller_manager_->read(time, period);
  this->controller_manager_->update(time, period);
  this->controller_manager_->write(time, period);
}

// <!-- #########jazzy########## -->
// 当前分支旧代码:
// void LeggedRos2Control::import_components_(
//   std::vector<hardware_interface::HardwareInfo> &hardware_info,
//   std::unique_ptr<hardware_interface::ResourceManager> &resource_manager)
// {
//   ...
// }
// <!-- #########jazzy########## -->



} // namespace legged
