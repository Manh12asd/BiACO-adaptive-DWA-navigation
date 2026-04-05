#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

class StandaloneCostmap : public rclcpp::Node
{
public:
  StandaloneCostmap() : Node("standalone_costmap_node")
  {
    // Khởi tạo Costmap2DROS
    // "local_costmap" là tên định danh để tìm trong file YAML
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap");

    // Kích hoạt Lifecycle tự động
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());

    RCLCPP_INFO(this->get_logger(), "Local Costmap với Obstacle và Inflation Layer đã sẵn sàng.");
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StandaloneCostmap>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}