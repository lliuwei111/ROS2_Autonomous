/**
 * Subscriber application.
 *
 * wei <542841336@qq.com>
 *
 * May 23, 2024
 */

#include <iostream>

#include <control.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //! For the options, we pass a default-constructed rvalue.
  auto subNode = std::make_shared<control::control>(rclcpp::NodeOptions());
  rclcpp::spin(subNode);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
