#include "mr_pf/particle_filter_node_parameter.hpp"
#include <filesystem>

ParticleFilterNodeParameter::ParticleFilterNodeParameter(rclcpp::Node &node) : n_(node)
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
void ParticleFilterNodeParameter::declare_default_parameter<int>(
    const std::string &name,
    int value_default,
    int min, int max, int step,
    const std::string &desription)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.integer_range = {range};
    descriptor.description = desription;
    n_.declare_parameter<int>(name, value_default, descriptor);
}

template <>
void ParticleFilterNodeParameter::declare_default_parameter<double>(
    const std::string &name,
    double value_default,
    double min, double max, double step,
    const std::string &desription)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.floating_point_range = {range};
    descriptor.description = desription;
    n_.declare_parameter<double>(name, value_default, descriptor);
}

template <>
void ParticleFilterNodeParameter::declare_default_parameter<std::string>(
    const std::string &name,
    const std::string &value_default,
    const std::string &desription)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = desription;
    n_.declare_parameter<std::string>(name, value_default, descriptor);
}

void ParticleFilterNodeParameter::declare_parameters()
{
    declare_default_parameter<int>("alevel", 10, 0, 10, 1, "level of implementation for students");
    declare_default_parameter<int>("parameter_update_cycle", 1000, 100, 10000, 1, "Parameter update rate in [ms]");
    declare_default_parameter<int>("nr_of_samples", 100, 1, 50000, 1, "Number of samples used");
    declare_default_parameter<std::string>("map_file", "", "map file");
    declare_default_parameter<double>("sensor_model.sigma_hit", 0.5, 0.001, 1., 0.001, "Standard distribution for likelihood field");
    declare_default_parameter<double>("sensor_model.z_hit", 0.7, 0., 5., 0.001, "z hit");
    declare_default_parameter<double>("sensor_model.z_short", 0.1, 0., 5., 0.001, "z short");
    declare_default_parameter<double>("sensor_model.z_rand", 0.4, 0., 1., 0.001, "z rand");
    declare_default_parameter<double>("sensor_model.z_max", 5, 0., 10., 0.001, "z max");
    declare_default_parameter<double>("sensor_model.rate_of_beams_used", 0.2, 0., 1., 0.001, "rate of beams used for weighting");
    declare_default_parameter<bool>("sensor_model.random_beams", false, "on true random beams are used");
    declare_default_parameter<bool>("motion_model.disable", false, "disable motion update");
    declare_default_parameter<double>("motion_model.duration_offset", 0.0, 0.0, 10., 0.1, "motion duration offset [s]");
    declare_default_parameter<double>("motion_model.alpha1", 0.3, 0., 10., 0.001, "motion noise alpha 1");
    declare_default_parameter<double>("motion_model.alpha2", 0.1, 0., 10., 0.001, "motion noise alpha 2");
    declare_default_parameter<double>("motion_model.alpha3", 0.3, 0., 10., 0.001, "motion noise alpha 3");
    declare_default_parameter<double>("motion_model.alpha4", 0.1, 0., 10., 0.001, "motion noise alpha 4");
    declare_default_parameter<double>("motion_model.alpha5", 0.3, 0., 10., 0.001, "motion noise alpha 5");
    declare_default_parameter<double>("motion_model.alpha6", 0.1, 0., 10., 0.001, "motion noise alpha 6");
    declare_default_parameter<bool>("resample.continues_reset", false, "continues resets");
    declare_default_parameter<double>("resample.rate", 0.05, 0., 1., 0.001, "motion_noise alpha 6");
    declare_default_parameter<std::string>("resample.strategy", "low variance", "use \'keep best\' or \'low variance\'");
    declare_default_parameter<double>("resample.noise_position", 0.0, 0., 10., 0.001, "static sigma position noise on colored samples");
    declare_default_parameter<double>("resample.noise_orientation", 0.0, 0., 10., 0.001, "static sigma orientation noise on colored samples");

    callback_update_parameters();
    using namespace std::chrono_literals;
    timer_update_parameter_ =
        n_.create_wall_timer(
            1ms * parameter_update_cycle_,
            std::bind(&ParticleFilterNodeParameter::callback_update_parameters, this));
}
void ParticleFilterNodeParameter::callback_update_parameters()
{
    std::string resample_strategy_str;
    int alevel;
    n_.get_parameter<int>("alevel", alevel);
    level = static_cast<Level>(alevel);
    n_.get_parameter<unsigned int>("parameter_update_cycle", parameter_update_cycle_);
    n_.get_parameter<unsigned int>("nr_of_samples", nr_of_samples);
    map_file = n_.get_parameter("map_file").get_parameter_value().get<std::string>();
    n_.get_parameter<double>("sensor_model.sigma_hit", sigma_hit);
    n_.get_parameter<double>("sensor_model.z_hit", z_hit);
    n_.get_parameter<double>("sensor_model.z_short", z_short);
    n_.get_parameter<double>("sensor_model.z_rand", z_rand);
    n_.get_parameter<double>("sensor_model.z_max", z_max);
    n_.get_parameter<double>("sensor_model.rate_of_beams_used", rate_of_beams_used);
    n_.get_parameter("sensor_model.random_beams", random_beams_used);
    n_.get_parameter("motion_model.disable", disable_update);
    n_.get_parameter("motion_model.duration_offset", duration_offset);
    n_.get_parameter("motion_model.alpha1", alpha1);
    n_.get_parameter("motion_model.alpha2", alpha2);
    n_.get_parameter("motion_model.alpha3", alpha3);
    n_.get_parameter("motion_model.alpha4", alpha4);
    n_.get_parameter("motion_model.alpha5", alpha5);
    n_.get_parameter("motion_model.alpha6", alpha6);
    resample_strategy_str = n_.get_parameter("resample.strategy").get_parameter_value().get<std::string>();
    n_.get_parameter("resample.continues_reset", continues_reset);
    n_.get_parameter("resample.rate", resample_rate);
    n_.get_parameter("resample.noise_position", resample_noise_position);
    n_.get_parameter("resample.noise_orientation", resample_noise_orientation);

    if (resample_strategy_str.compare("low variance") == 0)
        resample_strategy = LOW_VARIANCE;
    else if (resample_strategy_str.compare("keep best") == 0)
        resample_strategy = KEEP_BEST;
    else
    {
        resample_strategy = KEEP_BEST;
        RCLCPP_FATAL(n_.get_logger(), "The resample strategy %s is not supported", resample_strategy_str.c_str());
    }
    /**
     * @ToDo Sensor Model
     * Check if the map file exists and give out a RCLCPP_FATAL message. Hint use std::filesystem
     **/
    if (level > CHECK_MAP_FILE_EXISTS)
    {
    }
    else
    {
        /// @node your code
    }
    // RCLCPP_INFO(n_.get_logger(), "map_file: '%s'", this->map_file_.c_str());
    // RCLCPP_INFO(n_.get_logger(), "Publisher: '%d,%d'", this->map_width_pix_, this->map_height_pix_);
}
