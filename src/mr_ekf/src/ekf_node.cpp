
#include <chrono>
#include <thread>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "mr_ekf/ekf_node.hpp"
#include "mr_ekf/ekf_visualization.hpp"
#include "mr_ekf/ekf_node_parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/duration.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using std::placeholders::_1;
using namespace tuw;

EKFNode::EKFNode(rclcpp::NodeOptions options)
    : Node("ekf", options)
{
    /// create parameters
    param_ = std::make_shared<EKFNodeParameter>(*this);
    param_->declare_parameters();
    /// create the filter
    filter_ = std::make_shared<mr::EKFVisualization>();
    /// init filter with parameters
    filter_->init((mr::EKFParameter *)param_.get());
    filter_->load_map();

    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "Frame id of the vehicles base link";
        base_frame_ = this->declare_parameter<std::string>("base_link", "base_link", descriptor);
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "Frame id of the map";
        map_frame_ = this->declare_parameter<std::string>("map_link", "map", descriptor);
    }
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = "Filter update rate in [ms]";
        filter_update_cycle_ = this->declare_parameter<int>("filter_update_cycle", 100, descriptor);
    }

    sub_sync_laser_ = std::make_shared<message_filters::Subscriber<LaserScanMsg>>(this, "scan", rmw_qos_profile_default);
    sub_sync_line_segments_ = std::make_shared<message_filters::Subscriber<LineSegmentsMsg>>(this, "line_segments", rmw_qos_profile_default);
    sync_exact_ = std::make_shared<message_filters::Synchronizer<exact_policy>>(exact_policy(10), *sub_sync_line_segments_, *sub_sync_laser_);
    sync_exact_->registerCallback(&EKFNode::sync_callback, this);

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10, std::bind(&EKFNode::callback_cmd, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribed to cmd_vel");

    sub_ground_truth_ = create_subscription<nav_msgs::msg::Odometry>(
        "ground_truth",
        10, std::bind(&EKFNode::callback_ground_truth, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribed to ground_truth");

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(
        1ms * filter_update_cycle_, std::bind(&EKFNode::on_timer, this));

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    this->on_timer();
}

void EKFNode::sync_callback(const LineSegmentsMsg::SharedPtr msg_lines, const LaserScanMsg::SharedPtr msg_scan)
{
    callback_line_segments(msg_lines);
    callback_laser(msg_scan);
}

void EKFNode::callback_laser(const LaserScanMsg::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "callback_laser");
    Pose2D pose_sensor;                                                       /// sensor pose in base_frame
    StampedDataPtr<Points2D> z_s = std::make_shared<StampedData<Points2D>>(); /// measurements z in sensor frame

    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        return;
    }
    pose_sensor = Pose2D(t.transform.translation.x, t.transform.translation.y, QuaternionToYaw(t.transform.rotation));
    z_s->data.reserve(msg->ranges.size());
    z_s->tf = pose_sensor.tf();
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        double angle = msg->angle_min + msg->angle_increment * i;
        Polar2D beam(angle, msg->ranges[i]);
        Point2D p(beam.point());
        z_s->data.push_back(p);
    }
    filter_->correction(z_s);
}
void EKFNode::callback_line_segments(const LineSegmentsMsg::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "callback_laser");
    Pose2D pose_sensor;                                                                   /// sensor pose in base_frame
    StampedDataPtr<LineSegments2D> z_s = std::make_shared<StampedData<LineSegments2D>>(); /// measurements z in sensor frame

    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        return;
    }
    pose_sensor = Pose2D(t.transform.translation.x, t.transform.translation.y, QuaternionToYaw(t.transform.rotation));
    pose_sensor.recompute_cached_cos_sin();
    z_s->tf = pose_sensor.tf();

    for (size_t i = 0; i < msg->segments.size(); i++)
    {
        Point2D p0(msg->segments[i].p0.x,
                   msg->segments[i].p0.y);
        Point2D p1(msg->segments[i].p1.x,
                   msg->segments[i].p1.y);
        z_s->data.push_back(LineSegment2D(p0, p1));
    }
    filter_->correction(z_s);
}

void EKFNode::callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "callback_ground_truth");
    double yaw;
    QuaternionToYaw(msg->pose.pose.orientation, yaw);
    filter_->set_ground_truth(Pose2D(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw));
}

void EKFNode::callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "callback_cmd");
    if (!cmd_)
        cmd_ = std::make_shared<Command2D>();
    cmd_->set(msg->linear.x, msg->angular.z);
}

void EKFNode::on_timer()
{
    static rclcpp::Time time_call_last{this->now()};
    rclcpp::Time time_call_current = this->now();
    rclcpp::Duration d = time_call_current - time_call_last;
    time_call_last = time_call_current;
    filter_->prediction(cmd_, d.seconds());
}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(EKFNode)
