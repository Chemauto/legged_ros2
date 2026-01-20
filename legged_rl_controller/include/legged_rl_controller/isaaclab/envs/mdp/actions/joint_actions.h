/**
 * @file joint_actions.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "legged_rl_controller/isaaclab/envs/manager_based_rl_env.h"
#include "legged_rl_controller/isaaclab/manager/action_manager.h"

namespace isaaclab
{

class JointAction : public ActionTerm
{
public:
    JointAction(YAML::Node cfg, ManagerBasedRLEnv* env)
    : ActionTerm(cfg, env)
    {
        if (!cfg["joint_names"].IsDefined()) {
            throw std::runtime_error("Action config missing 'joint_names'.");
        }
        joint_names_ = cfg["joint_names"].as<std::vector<std::string>>();
        if (joint_names_.empty()) {
            throw std::runtime_error("Action config 'joint_names' is empty.");
        }
        action_dim_ = static_cast<int>(joint_names_.size());
        raw_actions_.assign(action_dim_, 0.0f);
        processed_actions_.assign(action_dim_, 0.0f);

        if (cfg["scale"].IsDefined() && !cfg["scale"].IsNull()) {
            if (cfg["scale"].IsSequence()) {
                scale_ = cfg["scale"].as<std::vector<float>>();
            } else {
                scale_.assign(action_dim_, cfg["scale"].as<float>());
            }
            if (scale_.size() != static_cast<size_t>(action_dim_)) {
                throw std::runtime_error("Action config 'scale' size mismatch.");
            }
        }
        if (cfg["offset"].IsDefined() && !cfg["offset"].IsNull()) {
            if (cfg["offset"].IsSequence()) {
                offset_ = cfg["offset"].as<std::vector<float>>();
            } else {
                offset_.assign(action_dim_, cfg["offset"].as<float>());
            }
            if (offset_.size() != static_cast<size_t>(action_dim_)) {
                throw std::runtime_error("Action config 'offset' size mismatch.");
            }
        }
    }

    void process_actions(std::vector<float> actions) override
    {
        if (actions.size() != static_cast<size_t>(action_dim_)) {
            throw std::runtime_error("Action size mismatch with joint_names.");
        }
        raw_actions_ = std::move(actions);
        for (int i = 0; i < action_dim_; ++i) {
            float value = raw_actions_[i];
            if (!scale_.empty()) {
                value *= scale_[i];
            }
            if (!offset_.empty()) {
                value += offset_[i];
            }
            processed_actions_[i] = value;
        }
    }


    int action_dim() override
    {
        return action_dim_;
    }

    std::vector<float> raw_actions() override
    {
        return raw_actions_;
    }
    
    std::vector<float> processed_actions() override
    {
        return processed_actions_;
    }

    void reset() override
    {
        raw_actions_.assign(action_dim_, 0.0f);
        processed_actions_.assign(action_dim_, 0.0f);
    }

protected:
    int action_dim_;
    std::vector<std::string> joint_names_;
    std::vector<float> raw_actions_;
    std::vector<float> processed_actions_;
    std::vector<float> scale_;
    std::vector<float> offset_;
};

// The name of the class must match the action name in IO_descriptors.yaml
// So the format does not obey the usual naming convention
class joint_position_action: public JointAction{
public:
    joint_position_action(YAML::Node cfg, ManagerBasedRLEnv* env)
    :JointAction(cfg, env)
    {
    }
};

REGISTER_ACTION(joint_position_action);

};
