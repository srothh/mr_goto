#include "mr_ekf/ekf_node_parameter.hpp"
#include <filesystem>

EKFNodeParameter::EKFNodeParameter(rclcpp::Node &node) : n_(node)
{

    name = n_.get_name();
    /// some parameters
    map_cols = 558;
    map_rows = 558;
    map_min_x = -9.;
    map_min_y = -9.;
    map_max_x = 9.;
    map_max_y = 9.;
    map_rotation = 0.;
    map_grid_x = 1.;
    map_grid_y = 1.;
}

template <>
void EKFNodeParameter::declare_default_parameter<int>(
    const std::string &name,
    int value_default,
    int min, int max, int step,
    const std::string &description)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.integer_range = {range};
    descriptor.description = description;
    n_.declare_parameter<int>(name, value_default, descriptor);
}

template <>
void EKFNodeParameter::declare_default_parameter<double>(
    const std::string &name,
    double value_default,
    double min, double max, double step,
    const std::string &description)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.floating_point_range = {range};
    descriptor.description = description;
    n_.declare_parameter<double>(name, value_default, descriptor);
}

template <>
void EKFNodeParameter::declare_default_parameter<std::string>(
    const std::string &name,
    const std::string &value_default,
    const std::string &description)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = description;
    n_.declare_parameter<std::string>(name, value_default, descriptor);
}

void EKFNodeParameter::declare_parameters()
{
    declare_default_parameter<int>("parameter_update_cycle", 100, 100, 10000, 1, "Parameter update rate in [ms]");
    declare_default_parameter<double>("init_sigma_location", 1., 0., 20., 0.001, "init sigma location");
    declare_default_parameter<double>("forward_prediction_time", 0.01, 0.001, 10., 0.001, "forward prediction time");
    declare_default_parameter<double>("init_sigma_orientation", 0.39, 0., 3.14, 0.001, "init sigma orientation");
    declare_default_parameter<bool>("enable_prediction", true, "enables prediction step");
    declare_default_parameter<bool>("enable_data_association", true, "enables data_association step");
    declare_default_parameter<bool>("enable_correction", true, "enables correction step");
    declare_default_parameter<double>("alpha_1", 3.0, 0., 10.0, 0.001, "motion noise alpha 1");
    declare_default_parameter<double>("alpha_2", 0.5, 0., 6.28, 0.001, "motion noise alpha 2");
    declare_default_parameter<double>("alpha_3", 2.0, 0., 10.0, 0.001, "motion noise alpha 3");
    declare_default_parameter<double>("alpha_4", 0.2, 0., 6.28, 0.001, "motion noise alpha 4");
    declare_default_parameter<double>("measurement_noise_rho", 5., 0., 10., 0.001, "measurement on lines rho");
    declare_default_parameter<double>("measurement_noise_alpha", 0.5, 0., 3.14, 0.001, "measurement on lines alpha");
    declare_default_parameter<double>("hough_space_pixel_rho", 558., 0., 1000., 0.001, "hough_space_size_rho");
    declare_default_parameter<double>("hough_space_pixel_alpha", 558., 0., 1000., 0.001, "hough_space_pixel_alpha");
    declare_default_parameter<double>("hough_space_meter_rho", 5., 0., 20., 0.001, "hough_space_meter_rho");
    declare_default_parameter<double>("data_association_line_rho", 0.5, 0., 1., 0.001, "threshold for line matching");
    declare_default_parameter<double>("data_association_line_alpha", 0.4, 0., 1.57, 0.001, "threshold for line matching");
    declare_default_parameter<double>("data_association_distance_to_endpoints", 4., 0., 5., 0.001, "squared distance to endpoints");
    declare_default_parameter<std::string>("map_linesegments_file", "", "line segments file");
    declare_default_parameter<std::string>("map_file", "", "map file");

    callback_update_parameters();
    using namespace std::chrono_literals;
    timer_update_parameter_ =
        n_.create_wall_timer(
            1ms * parameter_update_cycle_,
            std::bind(&EKFNodeParameter::callback_update_parameters, this));
}
void EKFNodeParameter::callback_update_parameters()
{
    n_.get_parameter<unsigned int>("parameter_update_cycle", parameter_update_cycle_);
    n_.get_parameter("forward_prediction_time", forward_prediction_time);
    n_.get_parameter("init_sigma_location", init_sigma_location);
    n_.get_parameter("init_sigma_orientation", init_sigma_orientation);
    n_.get_parameter("enable_prediction", enable_prediction);
    n_.get_parameter("enable_data_association", enable_data_association);
    n_.get_parameter("enable_correction", enable_correction);
    n_.get_parameter("alpha_1", alpha_1);
    n_.get_parameter("alpha_2", alpha_2);
    n_.get_parameter("alpha_3", alpha_3);
    n_.get_parameter("alpha_4", alpha_4);
    n_.get_parameter("measurement_noise_rho", measurement_noise_rho);
    n_.get_parameter("measurement_noise_alpha", measurement_noise_alpha);
    n_.get_parameter("hough_space_pixel_rho", hough_space_pixel_rho);
    n_.get_parameter("hough_space_pixel_alpha", hough_space_pixel_alpha);
    n_.get_parameter("hough_space_meter_rho", hough_space_meter_rho);
    n_.get_parameter("data_association_line_rho", data_association_line_rho);
    n_.get_parameter("data_association_line_alpha", data_association_line_alpha);
    n_.get_parameter("data_association_distance_to_endpoints", data_association_distance_to_endpoints);
    map_linesegments_file = n_.get_parameter("map_linesegments_file").get_parameter_value().get<std::string>();
    map_file = n_.get_parameter("map_file").get_parameter_value().get<std::string>();

    if (!std::filesystem::exists(map_linesegments_file))
    {
        RCLCPP_FATAL(
            n_.get_logger(), "The map_linesegments_file file %s does not exist.", map_linesegments_file.c_str());
        exit(0);
    }
    if (!std::filesystem::exists(map_file))
    {
        RCLCPP_FATAL(
            n_.get_logger(), "The map_file file %s does not exist.", map_file.c_str());
        exit(0);
    }
}
