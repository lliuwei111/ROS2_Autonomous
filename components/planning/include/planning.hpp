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

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <filesystem>

// 输入
#include "geometry_msgs/msg/pose_stamped.hpp" // 当前位置
#include "nav_msgs/msg/occupancy_grid.hpp" // 地图数据。
#include <std_msgs/msg/string.hpp>  // 目标位置或其他任务指令。

// 输出
#include "nav_msgs/msg/path.hpp"// 规划好的路径。
#include "action_msgs/msg/goal_status_array.hpp" // 动作执行状态。

//! There has to be a namespace when declaring a component class,
//! in order to avoid plugin name clashes with other components.
//! The name of the namespace should be the name of the package.
namespace planning
{

/**
 * Simple planning node: receives and prints strings transmitted on a topic.
 */
class planning : public rclcpp::Node
{
public:
  //! To be compatible with component containers, the constructor must have only this argument!
  planning(const rclcpp::NodeOptions & node_opts);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubscriber;
  void MsgCallback(const std_msgs::msg::String::SharedPtr msg);
  void ImageMsgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

} // namespace planning

#endif
