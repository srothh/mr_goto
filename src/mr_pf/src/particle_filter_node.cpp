
#include <chrono>
#include <thread>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "mr_pf/particle_filter_node.hpp"
#include "mr_pf/particle_filter_visualization.hpp"
#include "mr_pf/particle_filter_node_parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/duration.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using std::placeholders::_1;

PFNode::PFNode(rclcpp::NodeOptions options)
    : Node("pf", options)
{
    /// create parameters
    param_ = std::make_shared<ParticleFilterNodeParameter>(*this);
    param_->declare_parameters();
    /// create the filter
    filter_ = std::make_shared<mr::ParticleFilterVisualization>();
    /// init filter with paramters
    filter_->init(std::static_pointer_cast<mr::ParticleFilterParameter>(param_));

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
    sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10, std::bind(&PFNode::callback_laser, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribed to scan");

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10, std::bind(&PFNode::callback_cmd, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribed to cmd_vel");

    sub_ground_truth_ = create_subscription<nav_msgs::msg::Odometry>(
        "ground_truth",
        10, std::bind(&PFNode::callback_ground_truth, this, _1));
    RCLCPP_INFO(this->get_logger(), "subscribed to ground_truth");

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(
        1ms * filter_update_cycle_, std::bind(&PFNode::on_timer, this));

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    this->on_timer();
}

void PFNode::callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "callback_ground_truth");
    double yaw;
    tuw::QuaternionToYaw(msg->pose.pose.orientation, yaw);
    filter_->set_ground_truth(tuw::Pose2D(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw));
}

void PFNode::callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "callback_laser");
    tuw::Pose2D pose_sensor;                                /// sensor pose in base_frame
    std::vector<std::pair<tuw::Point2D, tuw::Polar2D>> z_s; /// measurements z in sensor frame

    /**
     * @ToDo Sensor Model
     * use the tf_buffer to catch the transform at the correct time and fill the pose_laser
     * compute the cartesian and polar coordinates for each beam and fill the vector z_s
     * hints:
     *  * use geometry_msgs::msg::TransformStamped
     *  * use tf2_ros::Buffer::lookupTransform
     *  * do not use hard coded frame id's!
     *  * use tuw::QuaternionToYaw
     *  * call tuw::Pose2D::recompute_cached_cos_sin on the the pose computed
     *  * Polar2D::point()
     **/
    if (param_->level > mr::ParticleFilterParameter::CATCH_LASER_TRANSFORMATION)
    {
    }
    else
    {
        /// @note your code here

        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
        double x = t.transform.rotation.x;
        double y = t.transform.rotation.y;
        double phi = tuw::QuaternionToYaw(t.transform.rotation);
        pose_sensor = tuw::Pose2D(x, y, phi);
        //pose_sensor.set(x, y, phi);
        
        pose_sensor.recompute_cached_cos_sin();
        z_s.resize(msg->ranges.size());
        for (size_t i = 0; i < msg->ranges.size(); i++){
            double theta = pose_sensor.get_theta() + msg->angle_min + i*msg->angle_increment;
            double r = sqrt(pose_sensor.get_x()*pose_sensor.get_x() + pose_sensor.get_y()*pose_sensor.get_y()) + msg->ranges[i];
            tuw::Polar2D beam(r, theta);
            tuw::Point2D p(beam);
            z_s[i] = std::pair<tuw::Point2D, tuw::Polar2D>(p, beam);
            //std::cout << "x: " << beam.get_x() << ", y:" << beam.get_y() << std::endl;
        }


        /// Below is dummy code, that should be deleted
        /*(void)msg; /// to silence a warning about unused variables
        for (double alpha = -M_PI; alpha < M_PI; alpha += M_PI / 100.)
        {
            tuw::Polar2D beam(alpha, 2.);
            pose_sensor.x() = 0.50;
            z_s.push_back(std::pair<tuw::Point2D, tuw::Polar2D>(beam.point(), beam));
        }*/
    }

    pose_sensor.recompute_cached_cos_sin();
    filter_->compute_weights(z_s, pose_sensor);
}

void PFNode::callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "callback_cmd");
    if (!cmd_)
        cmd_ = std::make_shared<tuw::Command2D>();
    cmd_->set(msg->linear.x, msg->angular.z);
}

void PFNode::on_timer()
{
    static rclcpp::Time time_call_last{this->now()};
    rclcpp::Time time_call_current = this->now();
    rclcpp::Duration d = time_call_current - time_call_last;
    time_call_last = time_call_current;
    filter_->resample(d.seconds());
    filter_->update(cmd_, d.seconds());
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(PFNode)
