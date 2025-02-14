/**
 * Subscriber component declaration.
 *
 * wei <542841336@qq.com>
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
// 输入
#include "nav_msgs/msg/path.hpp" // 来自规划模块的路径。
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

//! There has to be a namespace when declaring a component class,
//! in order to avoid plugin name clashes with other components.
//! The name of the namespace should be the name of the package.
namespace control
{

/**
 * Simple control node: receives and prints strings transmitted on a topic.
 */
class control : public rclcpp::Node
{
public:
  //! To be compatible with component containers, the constructor must have only this argument!
  control(const rclcpp::NodeOptions & node_opts);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubscriber;
  void MsgCallback(const std_msgs::msg::String::SharedPtr msg);
  void ImageMsgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

} // namespace control

#endif
