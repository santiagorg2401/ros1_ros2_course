#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher"), number_(4)
    {
        RCLCPP_INFO(this->get_logger(), "number_publisher C++ node initialized.");
        number_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                                std::bind(&NumberPublisherNode::publish_number, this));

        pub_ = this->create_publisher<std_msgs::msg::Int8>("number", 10);
    }

private:
    void publish_number()
    {
        auto msg = std_msgs::msg::Int8();
        msg.data = number_;
        pub_->publish(msg);
    }

    int number_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr number_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}