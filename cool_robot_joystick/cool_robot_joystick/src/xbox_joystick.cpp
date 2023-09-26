#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/char.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/set_bool.hpp"

class XboxJoystickNode : public rclcpp::Node
{
public:
    enum XboxButtons
    {
        LT = 0,
        LB,
        RT,
        RB,
        A,
        B,
        X,
        Y,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        BACK,
        START,
        LEFT_STICK,
        RIGHT_STICK,
        HOME,
        xbox_buttons_size
    };
    std::string buttonToString(XboxButtons button)
    {
        static const char *buttonStrings[] = {
            "LT", "LB", "RT", "RB", "A", "B", "X", "Y",
            "UP", "DOWN", "LEFT", "RIGHT", "BACK", "START", "LEFT_STICK", "RIGHT_STICK", "HOME"};

        if (button >= 0 && button < xbox_buttons_size)
        {
            return buttonStrings[button];
        }
        else
        {
            return "Unknown";
        }
    }

    XboxJoystickNode() : Node("xbox_joystick")
    {

        // pub

        // sub
        this->sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&XboxJoystickNode::onJoyCallback, this, std::placeholders::_1));

        // srv client
        this->client_servo = create_client<std_srvs::srv::SetBool>("/cool_robot_controller/servo");

        // 等待服務伺服器啟動
        while (!this->client_servo->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(get_logger(), "等待服務 %s 啟動...", this->client_servo->get_service_name());
        }

        // 按鍵
        this->btn_status.resize(xbox_buttons_size, false);
    }

private:
    // -------------------------------------
    // publisher
    // -------------------------------------

    // -------------------------------------
    // subscriber
    // -------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    void onJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 8 || msg->buttons.size() < 17)
            return;

        this->btn_status[XboxButtons::LT] = msg->axes[5] < 0.5 ? true : false;
        this->btn_status[XboxButtons::LB] = msg->buttons[6];
        this->btn_status[XboxButtons::RT] = msg->axes[4] < 0.5 ? true : false;
        this->btn_status[XboxButtons::RB] = msg->buttons[7];
        this->btn_status[XboxButtons::A] = msg->buttons[0];
        this->btn_status[XboxButtons::B] = msg->buttons[1];
        this->btn_status[XboxButtons::X] = msg->buttons[3];
        this->btn_status[XboxButtons::Y] = msg->buttons[4];
        this->btn_status[XboxButtons::UP] = msg->axes[7] > 0.5 ? true : false;
        this->btn_status[XboxButtons::DOWN] = msg->axes[7] < -0.5 ? true : false;
        this->btn_status[XboxButtons::LEFT] = msg->axes[6] > 0.5 ? true : false;
        this->btn_status[XboxButtons::RIGHT] = msg->axes[6] < -0.5 ? true : false;
        this->btn_status[XboxButtons::BACK] = msg->buttons[15];
        this->btn_status[XboxButtons::START] = msg->buttons[11];
        this->btn_status[XboxButtons::LEFT_STICK] = msg->buttons[13];
        this->btn_status[XboxButtons::RIGHT_STICK] = msg->buttons[14];
        this->btn_status[XboxButtons::HOME] = msg->buttons[16];

        if (this->last_btn_status.size() == this->btn_status.size())
        {
            // 檢測按鈕的狀態變化
            for (size_t idx = 0; idx < XboxButtons::xbox_buttons_size; idx++)
            {

                if (!this->last_btn_status[idx] && this->btn_status[idx])
                {
                    this->onKeyDown((XboxButtons)idx);
                }
                else if (this->last_btn_status[idx] && !this->btn_status[idx])
                {
                    this->onKeyUp((XboxButtons)idx);
                }
            }
        }

        this->last_btn_status = btn_status;
    }
    // -------------------------------------
    // service
    // -------------------------------------
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_servo;
    void client_servo_callback(const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        if (future.get()->success)
        {
            RCLCPP_INFO(get_logger(), "服務請求成功");
            RCLCPP_INFO(get_logger(), "Got result: [%s]", future.get()->message.c_str());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "服務請求失敗");
        }
    }

    // -------------------------------------
    // key events
    // -------------------------------------
    std::vector<bool> btn_status;
    std::vector<bool> last_btn_status;

    void onKeyDown(const XboxButtons button_key)
    {
        RCLCPP_INFO(get_logger(), "onKeyDown: %s", buttonToString(button_key).c_str());

        auto msg = std::make_shared<std_msgs::msg::Char>();

        if (button_key == XboxButtons::START)
        {
        }

        if (button_key == XboxButtons::LB)
        {
            // Servo On (ros2 service call /cool_robot_controller/servo std_srvs/srv/SetBool "data: True" )
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true; // servo on
            this->client_servo->async_send_request(request, std::bind(&XboxJoystickNode::client_servo_callback, this, std::placeholders::_1));
        }

        if (button_key == XboxButtons::LT)
        {
            // Servo Off (ros2 service call /cool_robot_controller/servo std_srvs/srv/SetBool "data: False" )
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = false; // servo off
            this->client_servo->async_send_request(request, std::bind(&XboxJoystickNode::client_servo_callback, this, std::placeholders::_1));

        }
    }

    void onKeyUp(const XboxButtons button_key)
    {
        RCLCPP_DEBUG(get_logger(), "onKeyUp: %s", buttonToString(button_key).c_str());
        (void)button_key;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XboxJoystickNode>());
    rclcpp::shutdown();
    return 0;
}