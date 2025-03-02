#include <rclcpp/rclcpp.hpp>
#include "pointcloud_server/RollingGrid.h"
#include <Eigen/Core>

class PointcloudServerNode : public rclcpp::Node
{
public:
  PointcloudServerNode()
  : Node("pointcloud_server_node")
  {
    RCLCPP_INFO(this->get_logger(), "Pointcloud Server Node Started");
    Eigen::Vector3f init_pos(0.0f, 0.0f, 0.0f);
    // Instantiate the RollingGrid using the initial position.
    rolling_grid_ = std::make_shared<LidarSlam::RollingGrid>(init_pos);
  }

private:
  std::shared_ptr<LidarSlam::RollingGrid> rolling_grid_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointcloudServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
