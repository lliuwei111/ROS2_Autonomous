/**
 * Subscriber component declaration.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * May 23, 2024
 */

#ifndef SUB_HPP
#define SUB_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <filesystem>

//! There has to be a namespace when declaring a component class,
//! in order to avoid plugin name clashes with other components.
//! The name of the namespace should be the name of the package.
namespace perception
{

/**
 * Simple Perception node: receives and prints strings transmitted on a topic.
 */
class Perception : public rclcpp::Node
{
public:
  //! To be compatible with component containers, the constructor must have only this argument!
  Perception(const rclcpp::NodeOptions & node_opts);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubscriber;
  void MsgCallback(const std_msgs::msg::String::SharedPtr msg);
};

} // namespace perception

#endif
