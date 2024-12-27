/**
 * Publisher component definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * May 23, 2024
 */

#include <camera.hpp>

//! There has to be a namespace when declaring a component class,
//! in order to avoid plugin name clashes with other components.
//! The name of the namespace should be the name of the package.
namespace camera
{

/**
 * @brief Creates a Camera node.
 */
Camera::Camera(const rclcpp::NodeOptions & node_opts)
: Node("camera_node", node_opts),
  mPubCnt(0)
{
  mPublisher = this->create_publisher<std_msgs::msg::String>(
    "/ros2_demo/string",
    rclcpp::QoS(10));

  mImagePublisher = this->create_publisher<sensor_msgs::msg::Image>(
    "/ros2_demo/image",
    rclcpp::QoS(10));

  mPubTimer = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD),
    std::bind(
      &Camera::PubTimerCallback,
      this));

  RCLCPP_INFO(this->get_logger(), "Publisher initialized");
}

/**
 * @brief Publishes a message on timer occurrence.
 */
void Camera::PubTimerCallback(void)
{
  // Build the new message
  std::string new_data = "Hello ";
  new_data.append(std::to_string(mPubCnt) + ".");

  std_msgs::msg::String new_msg{};

  new_msg.set__data(new_data);

  mPublisher->publish(new_msg);

  std::string path = "./image.jpg";
  cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Could not read the image.");
      return;
  }
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
  mImagePublisher->publish(*msg);
  RCLCPP_INFO(this->get_logger(), "Published image.");

  // Log something
  mPubCnt++;
  RCLCPP_INFO(this->get_logger(), "Published message %lu", mPubCnt);
}

} // namespace camera

//! Must do this at the end of one source file where your class definition is to generate the plugin.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(camera::Camera)
