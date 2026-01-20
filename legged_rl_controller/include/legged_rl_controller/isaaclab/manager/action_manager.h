/**
 * @file action_manager.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief Manager for action terms in a reinforcement learning environment.
 * @ref This file is adapted from unitree_rl_lab.
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>


namespace isaaclab {

class ManagerBasedRLEnv;  // forward declaration

class ActionTerm {
public:
  ActionTerm(YAML::Node cfg, ManagerBasedRLEnv *env)
  : cfg_(cfg), env_(env)
  {
  }

  virtual int action_dim() = 0;
  virtual std::vector<float> raw_actions() = 0;
  virtual std::vector<float> processed_actions() = 0;
  virtual void process_actions(std::vector<float> actions) = 0;
  virtual void reset() {}

protected:
  YAML::Node cfg_;
  ManagerBasedRLEnv *env_;
};

using ActionFactory =
  std::function<std::unique_ptr<ActionTerm>(YAML::Node, ManagerBasedRLEnv *)>;
using ActionMap = std::map<std::string, ActionFactory>;

inline ActionMap &actions_map()
{
  static ActionMap instance;
  return instance;
}

#define REGISTER_ACTION(name) \
  inline struct name##_registrar { \
    name##_registrar() { \
      actions_map()[#name] = [](YAML::Node cfg, ManagerBasedRLEnv *env) { \
        return std::make_unique<name>(cfg, env); \
      }; \
    } \
  } name##_registrar_instance;

class ActionManager {
public:
  ActionManager(YAML::Node cfg, ManagerBasedRLEnv *env)
  : cfg_(cfg), env_(env)
  {
    _prepare_terms();
    action_.resize(total_action_dim(), 0.0f);
  }

  void reset()
  {
    action_.assign(total_action_dim(), 0.0f);
    for (auto &term : terms_) {
      term->reset();
    }
  }

  std::vector<float> action()
  {
    return action_;
  }

  std::vector<float> processed_actions()
  {
    std::vector<float> actions;
    for (auto &term : terms_) {
      auto term_action = term->processed_actions();
      actions.insert(actions.end(), term_action.begin(), term_action.end());
    }
    return actions;
  }

  void process_action(std::vector<float> action)
  {
    action_ = action;
    int idx = 0;
    for (auto &term : terms_) {
      auto term_action = std::vector<float>(
        action.begin() + idx, action.begin() + idx + term->action_dim());
      term->process_actions(term_action);
      idx += term->action_dim();
    }
  }

  int total_action_dim()
  {
    auto dims = action_dim();
    return std::accumulate(dims.begin(), dims.end(), 0);
  }

  std::vector<int> action_dim()
  {
    std::vector<int> dims;
    for (auto &term : terms_) {
      dims.push_back(term->action_dim());
    }
    return dims;
  }

private:
  void _prepare_terms()
  {
    YAML::Node actions_node = cfg_["actions"];
    if (!actions_node.IsDefined()) {
      throw std::runtime_error("Action config missing 'actions' root node.");
    }
    if (!actions_node.IsSequence()) {
      throw std::runtime_error("Action config 'actions' must be a sequence.");
    }

    for (auto it = actions_node.begin(); it != actions_node.end(); ++it) {
      const auto term_yaml_cfg = *it;
      if (!term_yaml_cfg["name"].IsDefined()) {
        throw std::runtime_error("Action term missing 'name' field.");
      }
      std::string action_name = term_yaml_cfg["name"].as<std::string>();
      if (actions_map().find(action_name) == actions_map().end()) {
        throw std::runtime_error(
          "Action term '" + action_name + "' is not registered.");
      }

      auto term = actions_map()[action_name](term_yaml_cfg, env_);
      terms_.push_back(std::move(term));
    }
  }

  YAML::Node cfg_;
  ManagerBasedRLEnv *env_;
  std::vector<float> action_;
  std::vector<std::unique_ptr<ActionTerm>> terms_;
};

}  // namespace isaaclab
