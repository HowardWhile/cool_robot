// Copyright (c) 2023, howard
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cool_robot_controller/cool_robot_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

#define console(format, ...) \
    RCLCPP_INFO(get_node()->get_logger(), format, ##__VA_ARGS__)

#define console_preiod(period, format, ...) \
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), period, format, ##__VA_ARGS__)

namespace cool_robot_controller
{
    CoolRobotController::CoolRobotController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn CoolRobotController::on_init()
    {
        console("on_init()");

        control_mode_.initRT(control_mode_type::FAST);

        try
        {
            param_listener_ = std::make_shared<cool_robot_controller::ParamListener>(get_node());
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        // --------------------------------------
        // init publishers
        // --------------------------------------
        this->pub_status_words = this->get_node()->create_publisher<std_msgs::msg::UInt16MultiArray>("/status_words", 10);
        this->pub_control_words_state = this->get_node()->create_publisher<std_msgs::msg::UInt16MultiArray>("/control_words_state", 10);
        this->last_time_status_words_pub = this->get_node()->now();
        this->last_time_control_words_state_pub = this->get_node()->now();

        // --------------------------------------
        // init subscriber
        // --------------------------------------
        auto sub_control_word_callback = [this](std_msgs::msg::UInt16::UniquePtr msg) -> void
        {
            this->control_word = msg->data;
            this->control_word_renew = true;
            console("sub_control_word_callback");
        };
        this->sub_control_word = this->get_node()->create_subscription<std_msgs::msg::UInt16>("/control_word", 10, sub_control_word_callback);
        // --------------------------------------

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CoolRobotController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        params_ = param_listener_->get_params();
        console("on_configure()");

        int size = this->params_.joints.size();
        this->status_words.resize(size);
        this->control_words_state.resize(size);

        console("configure successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration CoolRobotController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        console("command_interface_configuration()");

        for (const std::string &j : this->params_.joints)
        {
            // console("%s", j.c_str());
            command_interfaces_config.names.push_back(j + "/control_word");
        }

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration CoolRobotController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        console("state_interface_configuration()");
        for (const std::string &j : this->params_.joints)
        {
            // console("%s", j.c_str());
            state_interfaces_config.names.push_back(j + "/status_word");
        }

        return state_interfaces_config;
    }

    controller_interface::CallbackReturn CoolRobotController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        console("on_activate()");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CoolRobotController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        console("on_deactivate()");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CoolRobotController::update(const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {
        // --------------------------------
        // Renew input
        // --------------------------------
        // 更新 status_words
        for (size_t idx = 0; idx < this->state_interfaces_.size(); idx++)
        {
            this->status_words[idx] = this->state_interfaces_[idx].get_value();
        }
        // console_preiod(1000, "status_word: %s", this->Join(", ", this->status_words).c_str());
        
        // 更新 control_words_state
        for (size_t idx = 0; idx < this->command_interfaces_.size(); idx++)
        {
            this->control_words_state[idx] = this->command_interfaces_[idx].get_value();
        }
        // console_preiod(1000, "control_words_state: %s", this->Join(", ", this->control_words_state).c_str());

        // --------------------------------
        // Logic control
        // --------------------------------
        // 有變化就發出topic
        bool enable_pub_status_words = false;
        if (this->hasVectorChanged(this->last_status_words, this->status_words) == true)
        {
            this->last_status_words = this->status_words;
            enable_pub_status_words = true;
        }

        // 有變化就發出topic
        bool enable_pub_control_words_state = false;
        if (this->hasVectorChanged(this->last_control_words_state, this->control_words_state) == true)
        {
            this->last_control_words_state = this->control_words_state;
            enable_pub_control_words_state = true;
        }

        // 沒有變化一段時間也發出topic
        if ((this->get_node()->now() - this->last_time_status_words_pub).seconds() > params_.topic_pub_interval)
        {
            this->last_time_status_words_pub = this->get_node()->now();
            enable_pub_status_words = true;
        }
        if ((this->get_node()->now() - this->last_time_control_words_state_pub).seconds() > params_.topic_pub_interval)
        {
            this->last_time_control_words_state_pub = this->get_node()->now();
            enable_pub_control_words_state = true;
        }

        //

        // --------------------------------
        // Update output
        // --------------------------------
        if (this->control_word_renew == true)
        {
            console("control_word_renew: %d (0x%X)", this->control_word, this->control_word);

            this->control_word_renew = false;
            for (size_t idx = 0; idx < this->command_interfaces_.size(); idx++)
            {
                this->command_interfaces_[idx].set_value(this->control_word);
            }
        }

        if (enable_pub_status_words)
        {
            auto msg = std::make_shared<std_msgs::msg::UInt16MultiArray>();
            msg->data = this->status_words;

            this->pub_status_words->publish(*msg);
            this->last_time_status_words_pub = this->get_node()->now();

            console("pub status_words: %s", this->Join(", ", this->status_words).c_str());

        }

        if (enable_pub_control_words_state)
        {
            auto msg = std::make_shared<std_msgs::msg::UInt16MultiArray>();
            msg->data = this->control_words_state;

            this->pub_control_words_state->publish(*msg);
            this->last_time_control_words_state_pub = this->get_node()->now();
            console("pub control_words_state: %s", this->Join(", ", this->control_words_state).c_str());
        }

        // ----------------------------------
        return controller_interface::return_type::OK;
        // ----------------------------------
    }

    std::string CoolRobotController::Join(std::string separator, std::vector<std::string> values)
    {
        std::string result;
        for (const std::string &word : values)
        {
            if (!result.empty())
            {
                result += separator;
            }
            result += word;
        }
        return result;
    }

    std::string CoolRobotController::Join(std::string separator, std::vector<int> values)
    {
        std::vector<std::string> str_valuse;
        for (const auto &data : values)
        {
            str_valuse.push_back(std::to_string(data));
        }
        return this->Join(separator, str_valuse);
    }

    std::string CoolRobotController::Join(std::string separator, std::vector<uint16_t> values)
    {
        std::vector<std::string> str_valuse;
        for (const auto &data : values)
        {
            str_valuse.push_back(std::to_string(data));
        }
        return this->Join(separator, str_valuse);
    }

    bool CoolRobotController::hasVectorChanged(const std::vector<uint16_t> &previous, const std::vector<uint16_t> &current)
    {
        return previous != current;
    }

} // namespace cool_robot_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    cool_robot_controller::CoolRobotController, controller_interface::ControllerInterface)
