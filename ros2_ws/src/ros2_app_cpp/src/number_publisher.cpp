#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "number_publisher C++ node initialized.");
        this->declare_parameter("pub_freq", 1.0);
        this->declare_parameter("num2pub", 0);

        number_ = this->get_parameter("num2pub").as_int();
        pub_freq_ = this->get_parameter("pub_freq").as_double();

        number_timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000.0 / pub_freq_)),
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
    double pub_freq_;
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