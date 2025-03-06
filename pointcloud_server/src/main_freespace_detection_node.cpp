#include "rclcpp/rclcpp.hpp"
#include "pointcloud_server/freespace_detection_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<freespace_detection::FreespaceDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
