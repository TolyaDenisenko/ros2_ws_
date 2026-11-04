// Ретранслятор geometry_msgs/Twist → geometry_msgs/TwistStamped.
// Нужен контроллерам ros2_control (diff_drive_controller в Jazzy+ принимает
// только TwistStamped), когда источник команд (teleop, старые ноды) шлёт Twist.
//
// Параметры:
//   input_topic  (по умолчанию /cmd_vel_unstamped)
//   output_topic (по умолчанию /diff_drive_controller/cmd_vel)
//   frame_id     (по умолчанию base_link)

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class TwistToTwistStamped : public rclcpp::Node
{
public:
  TwistToTwistStamped()
  : Node("twist_to_twist_stamped")
  {
    const auto input = declare_parameter("input_topic", "/cmd_vel_unstamped");
    const auto output = declare_parameter("output_topic", "/diff_drive_controller/cmd_vel");
    frame_id_ = declare_parameter("frame_id", "base_link");

    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output, 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input, 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        geometry_msgs::msg::TwistStamped out;
        out.header.stamp = now();
        out.header.frame_id = frame_id_;
        out.twist = *msg;
        pub_->publish(out);
      });

    RCLCPP_INFO(get_logger(), "%s (Twist) → %s (TwistStamped)", input.c_str(), output.c_str());
  }

private:
  std::string frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToTwistStamped>());
  rclcpp::shutdown();
  return 0;
}
