#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "my_robot_interfaces/srv/reset_counter.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "number_counter C++ node has been initialized.");
        sub_ = this->create_subscription<std_msgs::msg::Int8>("number", 10, std::bind(&NumberCounterNode::callback_number, this, _1));
        reset_srv = this->create_service<my_robot_interfaces::srv::ResetCounter>("reset_counter",
                                                                                 std::bind(&NumberCounterNode::reset_counter_callback,
                                                                                           this, _1, _2));
    }

private:
    void callback_number(const std_msgs::msg::Int8 msg)
    {
        counter_ += msg.data;
        RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
    }
    void reset_counter_callback(const my_robot_interfaces::srv::ResetCounter::Request::SharedPtr request,
                                const my_robot_interfaces::srv::ResetCounter::Response::SharedPtr response)
    {
        if (request->reset_value >= 0)
        {
            counter_ = request->reset_value;
            RCLCPP_INFO(this->get_logger(), "Reset service activated, counter value is %d", counter_);
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
    rclcpp::Service<my_robot_interfaces::srv::ResetCounter>::SharedPtr reset_srv;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
