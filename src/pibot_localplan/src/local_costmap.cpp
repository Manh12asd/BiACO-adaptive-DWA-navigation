#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);


  auto costmap_node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");

  costmap_node->configure();
  costmap_node->activate();

  RCLCPP_INFO(costmap_node->get_logger(), "Local Costmap với Obstacle và Inflation Layer đã sẵn sàng.");

  rclcpp::spin(costmap_node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}