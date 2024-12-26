/**
 * Subscriber application.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * May 23, 2024
 */

#include <iostream>

#include <perception.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //! For the options, we pass a default-constructed rvalue.
  auto subNode = std::make_shared<perception::Perception>(rclcpp::NodeOptions());
  rclcpp::spin(subNode);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
