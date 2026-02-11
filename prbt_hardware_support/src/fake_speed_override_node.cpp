/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Migrated to ROS2 Jazzy.
 * Provides a fake speed override service that always returns 1.0.
 */

#include <rclcpp/rclcpp.hpp>
#include <pilz_msgs/srv/get_speed_override.hpp>

class FakeSpeedOverrideNode : public rclcpp::Node
{
public:
  FakeSpeedOverrideNode() : Node("fake_speed_override_node")
  {
    speed_override_ = this->declare_parameter("speed_override", 1.0);

    service_ = this->create_service<pilz_msgs::srv::GetSpeedOverride>(
      "/prbt/get_speed_override",
      [this](const std::shared_ptr<pilz_msgs::srv::GetSpeedOverride::Request> /*request*/,
             std::shared_ptr<pilz_msgs::srv::GetSpeedOverride::Response> response)
      {
        response->speed_override = speed_override_;
      });

    RCLCPP_INFO(this->get_logger(), "Fake speed override node started (speed_override=%.2f)", speed_override_);
  }

private:
  rclcpp::Service<pilz_msgs::srv::GetSpeedOverride>::SharedPtr service_;
  double speed_override_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeSpeedOverrideNode>());
  rclcpp::shutdown();
  return 0;
}
