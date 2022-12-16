#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


class VelPublisher : public rclcpp::Node
{
  public:
    VelPublisher()
    : Node("vel_pub"), count_(0)
    {
      vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&VelPublisher::timer_callback, this));

    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      if (count_ < 1) {
        message.linear.x = 0.0;
        message.angular.z = 0.0;
      } 
      else {
        message.linear.x = 0.3; // m/s
        message.angular.z = 0.3; // rad/s
      }
      count_++;    
      vel_publisher->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelPublisher>());
  rclcpp::shutdown();
  return 0;
}