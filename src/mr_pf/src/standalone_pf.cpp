#include <memory>
#include "mr_pf/particle_filter_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
