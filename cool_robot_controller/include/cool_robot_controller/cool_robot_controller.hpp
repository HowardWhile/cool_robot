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

#ifndef COOL_ROBOT_CONTROLLER__COOL_ROBOT_CONTROLLER_HPP_
#define COOL_ROBOT_CONTROLLER__COOL_ROBOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "cool_robot_controller_parameters.hpp"
#include "cool_robot_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace cool_robot_controller
{
    // name constants for state interfaces
    static constexpr size_t STATE_MY_ITFS = 0;

    // name constants for command interfaces
    static constexpr size_t CMD_MY_ITFS = 0;

    // TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
    enum class control_mode_type : std::uint8_t
    {
        FAST = 0,
        SLOW = 1,
    };

    class CoolRobotController : public controller_interface::ControllerInterface
    {
    public:
        COOL_ROBOT_CONTROLLER__VISIBILITY_PUBLIC
        CoolRobotController();

        COOL_ROBOT_CONTROLLER__VISIBILITY_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        COOL_ROBOT_CONTROLLER__VISIBILITY_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        COOL_ROBOT_CONTROLLER__VISIBILITY_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        COOL_ROBOT_CONTROLLER__VISIBILITY_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        COOL_ROBOT_CONTROLLER__VISIBILITY_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        COOL_ROBOT_CONTROLLER__VISIBILITY_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        COOL_ROBOT_CONTROLLER__VISIBILITY_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // TODO(anyone): replace the state and command message types
        using ControllerModeSrvType = std_srvs::srv::SetBool;

    protected:
        std::shared_ptr<cool_robot_controller::ParamListener> param_listener_;
        cool_robot_controller::Params params_;

        // Command subscribers and Controller State publisher
        rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
        realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

        // publishers
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_status_words;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_control_words_state;

        // subscribers
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_control_word;

        // services
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_servo;
        void srv_servo_callback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    private:
        std::vector<uint16_t> status_words;
        std::vector<uint16_t> last_status_words;
        rclcpp::Time last_time_status_words_pub;

        std::vector<uint16_t> control_words_state;
        std::vector<uint16_t> last_control_words_state;
        rclcpp::Time last_time_control_words_state_pub;

        uint16_t control_word; //
        bool control_word_renew = false;

        // servo control
        bool request_servo_off = false;
        bool request_servo_on = false;
        int servo_on_step = 0;
        int servo_on_work();


        // 為每個項目或成員之間加入指定的分隔符號
        std::string Join(std::string separator, std::vector<std::string> values);
        std::string Join(std::string separator, std::vector<int> values);
        std::string Join(std::string separator, std::vector<uint16_t> values);

        // 函數用於檢查向量是否有變化
        bool hasVectorChanged(const std::vector<uint16_t> &previous, const std::vector<uint16_t> &current);

        // 型態轉換
        short bool2short(const bool bool16[16]);
    };

} // namespace cool_robot_controller

#endif // COOL_ROBOT_CONTROLLER__COOL_ROBOT_CONTROLLER_HPP_
