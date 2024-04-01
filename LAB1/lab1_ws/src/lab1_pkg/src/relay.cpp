#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// used in the bind method and means that the callback has 1 argument
using std::placeholders::_1;

class RelaySubscriber : public rclcpp::Node
{
  public:

    // the Node object is an attribute of the RelaySubscriber class, and we can initialize it this way instead inside the constructor
    RelaySubscriber() : Node("relay")  
    {
      // create_subsription creates a subscriber to the topic "drive" and the message type AckermannDriveStamped
      // 10 is the quality of service, denotes a queue that can store messages in case the subscriber cannot keep up with the publisher
      subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "drive", 10, std::bind(&RelaySubscriber::topic_callback, this, _1));

      publisher_relay = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
    }

  private:
    // Class properties
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_relay;

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


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelaySubscriber>());
  rclcpp::shutdown();
  return 0;
}