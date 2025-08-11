#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
member function as a callback from the timer. */
class SimplePublisher : public::rclcpp::Node
{
public:
    SimplePublisher(): Node("simple_publisher"), count_(0)
    {
        pub_  = create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timer_callaback, this));
    }

private:
    unsigned int count_;
    rclcpp::TimerBase::SharedPtr timer_ ;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ ;

    void timer_callaback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        pub_->publish(message);
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return (0);
}