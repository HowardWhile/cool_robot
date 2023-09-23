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

namespace
{ // utility

    // TODO(destogl): remove this when merged upstream
    // Changed services history QoS to keep all so we don't lose any client service calls
    static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
        RMW_QOS_POLICY_HISTORY_KEEP_ALL,
        1, // message queue depth
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false};

    using ControllerReferenceMsg = cool_robot_controller::CoolRobotController::ControllerReferenceMsg;

    // called from RT control loop
    void reset_controller_reference_msg(
        std::shared_ptr<ControllerReferenceMsg> &msg, const std::vector<std::string> &joint_names)
    {
        msg->joint_names = joint_names;
        msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
        msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
        msg->duration = std::numeric_limits<double>::quiet_NaN();
    }

} // namespace

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

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CoolRobotController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        params_ = param_listener_->get_params();
        console("on_configure()");
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
            console("%s", j.c_str());
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
            console("%s", j.c_str());
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
        {
            static uint32_t fps_count = 0;
            static uint32_t fps = 0;
            fps_count++;
            static auto last_check_time = rclcpp::Clock().now();
            auto dt = rclcpp::Clock().now() - last_check_time;
            if (dt.seconds() >= 1.0)
            {
                last_check_time = rclcpp::Clock().now();
                fps = fps_count;
                fps_count = 0;
            }

            // console_preiod(1000, "update() fps: %d", fps);
        }

        // std::vector<std::string> interface_names;
        std::vector<int> control_words;
        for (size_t idx=  0; idx < this->command_interfaces_.size(); idx++)
        {
            this->command_interfaces_[idx].set_value(0x1);  
        }
        
        for (const auto &c : this->command_interfaces_)
        {
            uint16_t value = c.get_value();
            control_words.push_back(value);
        }
        // console("%s", this->Join(", ", interface_names).c_str());
        console_preiod(1000, "control_words: %s", this->Join(", ", control_words).c_str());

        std::vector<int> status_words;
        for (const auto &s : this->state_interfaces_)
        {
            uint16_t value = s.get_value();
            status_words.push_back(value);
        }
        console_preiod(1000, "status_word: %s", this->Join(", ", status_words).c_str());

        return controller_interface::return_type::OK;
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
} // namespace cool_robot_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    cool_robot_controller::CoolRobotController, controller_interface::ControllerInterface)
