#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber() : Node("relay")
    {
      subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "drive", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      publisher_relay = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
    }

  private:
    void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.drive.speed);
      
      auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

      drive_msg.header.stamp = this->now();
      drive_msg.header.frame_id = "drive_frame";

      drive_msg.drive.speed = msg.drive.speed * 3;
      drive_msg.drive.steering_angle = msg.drive.steering_angle * 3;

      RCLCPP_INFO(this->get_logger(), "I published: '%f'", drive_msg.drive.speed);

      publisher_relay->publish(drive_msg);
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_relay;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}