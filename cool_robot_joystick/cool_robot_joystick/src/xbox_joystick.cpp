#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/char.hpp>
#include <rclcpp/rclcpp.hpp>

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

    // key events
    std::vector<bool> btn_status;
    std::vector<bool> last_btn_status;

    void onKeyDown(const XboxButtons button_key)
    {
        RCLCPP_INFO(get_logger(), "onKeyDown: %s", buttonToString(button_key).c_str());


        auto msg = std::make_shared<std_msgs::msg::Char>();

        if(button_key == XboxButtons::START)
        {
        }

        if(button_key == XboxButtons::BACK)
        {
        }

        if(button_key == XboxButtons::A)
        {
        }

        if(button_key == XboxButtons::B)
        {
        }

        if(button_key == XboxButtons::X)
        {
        }

        if(button_key == XboxButtons::Y)
        {
        }

        if(button_key == XboxButtons::RB)
        {
        }

        if(button_key == XboxButtons::UP)
        {
        }

        if(button_key == XboxButtons::DOWN)
        {
        }

        if(button_key == XboxButtons::LEFT_STICK)
        {
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