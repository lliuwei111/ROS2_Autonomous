/**
 * planning component definition.
 *
 * wei <542841336@qq.com>
 *
 * May 23, 2024
 */

#include <planning.hpp>

//! There has to be a namespace when declaring a component class,
//! in order to avoid plugin name clashes with other components.
//! The name of the namespace should be the name of the package.
namespace planning
{

/**
 * @brief Creates a planning node.
 */
planning::planning(const rclcpp::NodeOptions & node_opts)
: Node("planning_node", node_opts)
{
  mSubscriber = this->create_subscription<std_msgs::msg::String>(
    "/ros2_demo/string",
    rclcpp::QoS(10),
    std::bind(
      &planning::MsgCallback,
      this,
      std::placeholders::_1));
  mImageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
    "/ros2_demo/Image",
    rclcpp::QoS(10),
    std::bind(
      &planning::ImageMsgCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "planning initialized");
}

/**
 * @brief Echoes a new message.
 *
 * @param msg New message.
 */
void planning::MsgCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), msg->data.c_str());
}

void planning::ImageMsgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), msg->data.c_str());
}

} // namespace planning

//! Must do this at the end of one source file where your class definition is to generate the plugin.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning::planning)
