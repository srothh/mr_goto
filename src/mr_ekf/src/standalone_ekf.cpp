#include <memory>
#include "mr_ekf/ekf_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
