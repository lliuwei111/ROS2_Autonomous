/**
 * CAMERA component declaration.
 *
 * wei <542841336@qq.com>
 *
 * May 23, 2024
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/image.hpp" // 相机内参信息。
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <filesystem>

#define PUB_PERIOD 300 // Publisher transmission time period [ms]

//! There has to be a namespace when declaring a component class,
//! in order to avoid plugin name clashes with other components.
//! The name of the namespace should be the name of the package.
namespace camera
{

/**
 * Simple Camera node: transmits strings on a topic.
 */
class Camera : public rclcpp::Node
{
public:
  //! To be compatible with component containers, the constructor must have only this argument!
  Camera(const rclcpp::NodeOptions & node_opts);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePublisher;

  rclcpp::TimerBase::SharedPtr mPubTimer;
  void PubTimerCallback(void);

  unsigned long mPubCnt; // Marks messages
};

} // namespace camera

#endif
