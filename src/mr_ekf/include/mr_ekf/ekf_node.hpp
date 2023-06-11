#ifndef MR_POSE_ESTIMATION_PKG__EKF_NODE_HPP_
#define MR_POSE_ESTIMATION_PKG__EKF_NODE_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tuw_geometry/tuw_geometry.hpp>
#include <tuw_geometry_msgs/msg/line_segments.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

namespace mr
{
  class EKF;
}
class EKFNodeParameter;

using LineSegmentsMsg = tuw_geometry_msgs::msg::LineSegments;
using LaserScanMsg = sensor_msgs::msg::LaserScan;

/**
 * ROS2 Node which forward incoming and outgoing data to the particle filter
 */
class EKFNode : public rclcpp::Node
{
public:
  /// Constructor
  __attribute__((visibility("default"))) EKFNode(rclcpp::NodeOptions options);

private:
  rclcpp::TimerBase::SharedPtr timer_;                                        /// timer to on_time()
  std::shared_ptr<cv::Vec<double, 3>> p_lclick_;                              /// if allocated a left clicked point
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;        /// motion command subscriber
  tuw::Command2DPtr cmd_;                                                     /// local copy of the last command
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_; /// subscriber ground truth pose (simulation)
  nav_msgs::msg::Odometry::SharedPtr ground_truth_;                           /// local copy of the last ground truth pose
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};          /// tf listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                /// tf buffer
  std::string base_frame_;                                                    /// name of the robots base frame id
  std::string map_frame_;                                                     /// name of the map frame id
  unsigned int filter_update_cycle_;                                          /// parameter to define the update cycle time in ms
  rclcpp::Subscription<LineSegmentsMsg>::SharedPtr sub_line_segments_;        /// line subscriber
  rclcpp::Subscription<LaserScanMsg>::SharedPtr sub_laser_;                   /// laser subscriber

  typedef message_filters::sync_policies::ExactTime<LineSegmentsMsg, LaserScanMsg> exact_policy;
  std::shared_ptr<message_filters::Subscriber<LineSegmentsMsg>> sub_sync_line_segments_;
  std::shared_ptr<message_filters::Subscriber<LaserScanMsg>> sub_sync_laser_;
  std::shared_ptr<message_filters::Synchronizer<exact_policy>> sync_exact_;

  void sync_callback(const LineSegmentsMsg::SharedPtr msg_lines, const LaserScanMsg::SharedPtr msg_scan);

  /// Callback function for incoming range measurements
  void callback_laser(const LaserScanMsg::SharedPtr msg);

  /// Callback function for incoming line segments
  void callback_line_segments(const LineSegmentsMsg::SharedPtr msg); /// Callback line_segments

  /// Callback function for incoming twist command measurements
  void callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);

  /// Callback function for incoming Odometry ground truth msgs
  void callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// function called by an internal callback x time per second
  void on_timer();

  std::shared_ptr<mr::EKF> filter_;         /// pointer to the actual particle filter
  std::shared_ptr<EKFNodeParameter> param_; /// pointer to the parameters used by the particle filter

};

#endif // MR_POSE_ESTIMATION_PKG__EKF_NODE_HPP_
