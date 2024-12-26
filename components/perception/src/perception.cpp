/**
 * Perception component definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * May 23, 2024
 */

#include <perception.hpp>

//! There has to be a namespace when declaring a component class,
//! in order to avoid plugin name clashes with other components.
//! The name of the namespace should be the name of the package.
namespace perception
{

/**
 * @brief Creates a Perception node.
 */
Perception::Perception(const rclcpp::NodeOptions & node_opts)
: Node("perception_node", node_opts)
{
  mSubscriber = this->create_subscription<std_msgs::msg::String>(
    "/ros2_demo/string",
    rclcpp::QoS(10),
    std::bind(
      &Perception::MsgCallback,
      this,
      std::placeholders::_1));
  mImageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
    "/ros2_demo/Image",
    rclcpp::QoS(10),
    std::bind(
      &Perception::ImageMsgCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Perception initialized");
}

/**
 * @brief Echoes a new message.
 *
 * @param msg New message.
 */
void Perception::MsgCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), msg->data.c_str());
}

void Perception::ImageMsgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), msg->data.c_str());
}

} // namespace perception

//! Must do this at the end of one source file where your class definition is to generate the plugin.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception::Perception)
