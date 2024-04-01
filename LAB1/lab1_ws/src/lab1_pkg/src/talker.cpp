#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// to be able to use ms
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TalkerPublisher : public rclcpp::Node
{
  public:
    TalkerPublisher() : Node("talker")
    {
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
      
      this->declare_parameter("v", rclcpp::PARAMETER_DOUBLE);
      this->declare_parameter("d", rclcpp::PARAMETER_DOUBLE);
       
      timer_ = this->create_wall_timer(0ms, std::bind(&TalkerPublisher::timer_callback, this));
    }

  private:
    // class attributes
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
      auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

      drive_msg.header.stamp = this->now();
      drive_msg.header.frame_id = "drive_frame";

      drive_msg.drive.speed = this->get_parameter("v").as_double();
      drive_msg.drive.steering_angle = this->get_parameter("d").as_double();  

      RCLCPP_INFO(this->get_logger(), "Publishing talker");
      publisher_->publish(drive_msg);
    }

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerPublisher>());
  rclcpp::shutdown();
  return 0;
}