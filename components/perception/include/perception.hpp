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

// 感知输入
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp" // 位姿估计消息
#include "sensor_msgs/msg/point_cloud2.hpp" // 经过处理后的点云（例如分割后的点云）也是输出

// 感知输出
#include "vision_msgs/msg/detection2_d_array.hpp" // 2D 检测结果
#include "geometry_msgs/msg/pose_array.hpp" //  检测到的对象姿态集合。

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
  void ImageMsgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

} // namespace perception

#endif
