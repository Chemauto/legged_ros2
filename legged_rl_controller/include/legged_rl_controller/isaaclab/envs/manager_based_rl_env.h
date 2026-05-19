/**
 * @file manager_based_rl_env.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief A reinforcement learning environment based on observation and action managers.
 * @ref This file is adapted from unitree_rl_lab.
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "legged_rl_controller/isaaclab/algorithms/algorithms.h"
#include "legged_rl_controller/isaaclab/assets/articulation/articulation.h"
#include "legged_rl_controller/isaaclab/manager/observation_manager.h"
#include "legged_rl_controller/isaaclab/manager/action_manager.h"

namespace isaaclab {

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv {
public:
  ManagerBasedRLEnv(YAML::Node cfg, std::shared_ptr<Articulation> robot_)
  : cfg(cfg), robot(std::move(robot_))
  {
    const auto scene_node = cfg["scene"];
    if (!scene_node.IsDefined() || !scene_node["dt"].IsDefined()) {
      throw std::runtime_error("Config missing 'scene.dt'.");
    }
    step_dt = scene_node["dt"].as<float>();

    const auto articulations_node = cfg["articulations"];
    if (!articulations_node.IsDefined() || !articulations_node["robot"].IsDefined()) {
      throw std::runtime_error("Config missing 'articulations.robot'.");
    }
    const auto robot_node = articulations_node["robot"];
    if (!robot_node["joint_names"].IsDefined()) {
      throw std::runtime_error("Config missing 'articulations.robot.joint_names'.");
    }
    const auto joint_names = robot_node["joint_names"].as<std::vector<std::string>>();
    robot->data.joint_names = joint_names;
    robot->data.joint_pos.resize(joint_names.size());
    robot->data.joint_vel.resize(joint_names.size());

    {  // default joint positions and velocities
      if (!robot_node["default_joint_pos"].IsDefined()) {
        throw std::runtime_error("Config missing 'articulations.robot.default_joint_pos'.");
      }
      auto default_joint_pos = robot_node["default_joint_pos"].as<std::vector<float>>();
      if (default_joint_pos.size() != joint_names.size()) {
        throw std::runtime_error(
          "Size mismatch: default_joint_pos vs joint_names.");
      }
      robot->data.default_joint_pos =
        Eigen::VectorXf::Map(default_joint_pos.data(), default_joint_pos.size());

      if (!robot_node["default_joint_vel"].IsDefined()) {
        throw std::runtime_error("Config missing 'articulations.robot.default_joint_vel'.");
      }
      auto default_joint_vel = robot_node["default_joint_vel"].as<std::vector<float>>();
      if (default_joint_vel.size() != joint_names.size()) {
        throw std::runtime_error(
          "Size mismatch: default_joint_vel vs joint_names.");
      }
      robot->data.default_joint_vel =
        Eigen::VectorXf::Map(default_joint_vel.data(), default_joint_vel.size());
    }
    {  // joint stiffness and damping
      if (!robot_node["default_joint_stiffness"].IsDefined()) {
        throw std::runtime_error("Config missing 'articulations.robot.default_joint_stiffness'.");
      }
      if (!robot_node["default_joint_damping"].IsDefined()) {
        throw std::runtime_error("Config missing 'articulations.robot.default_joint_damping'.");
      }
      robot->data.joint_stiffness = robot_node["default_joint_stiffness"].as<std::vector<float>>();
      robot->data.joint_damping = robot_node["default_joint_damping"].as<std::vector<float>>();
      if (robot->data.joint_stiffness.size() != joint_names.size()) {
        throw std::runtime_error(
          "Size mismatch: default_joint_stiffness vs joint_names.");
      }
      if (robot->data.joint_damping.size() != joint_names.size()) {
        throw std::runtime_error(
          "Size mismatch: default_joint_damping vs joint_names.");
      }
    }

    // Load managers.
    action_manager = std::make_unique<ActionManager>(cfg, this);
    observation_manager = std::make_unique<ObservationManager>(cfg, this);
  }

  void reset()
  {
    global_phase = 0.0f;
    episode_length = 0;
    robot->update();
    action_manager->reset();
    observation_manager->reset();
  }

  void step()
  {
    episode_length += 1;
    robot->update();
    auto obs = observation_manager->compute();
    auto action = alg->act(obs);
    action_manager->process_action(action);
  }

  float step_dt;
  YAML::Node cfg;
  std::unique_ptr<ObservationManager> observation_manager;
  std::unique_ptr<ActionManager> action_manager;
  std::shared_ptr<Articulation> robot;
  std::unique_ptr<Algorithms> alg;
  long episode_length = 0;
  float global_phase = 0.0f;
};

}  // namespace isaaclab
