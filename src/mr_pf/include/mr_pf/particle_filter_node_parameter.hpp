#ifndef MR_PF_PKG__PARTICLE_FILTER_NODE_PARAMETER_HPP_
#define MR_PF_PKG__PARTICLE_FILTER_NODE_PARAMETER_HPP_
#include <rclcpp/rclcpp.hpp>
#include <mr_pf/particle_filter_parameter.hpp>

/**
 * class to handle particle filter parameter using ros parameter callbacks
 */
class ParticleFilterNodeParameter : public mr::ParticleFilterParameter
{
public:
    /**
     * Constructor
     * @param node reference to the parent node
     */
    ParticleFilterNodeParameter(rclcpp::Node &node);
    void declare_parameters();         /// declares ros parameters
    void callback_update_parameters(); /// callback to check changes on the parameters
private:
    /**
     * Helper to declare numbered parameter
     * @param name
     * @param value_default
     * @param min
     * @param max
     * @param step
     * @param description
     */
    template <typename T>
    void declare_default_parameter(
        const std::string &name,
        T value_default,
        T min, T max, T step,
        const std::string &description);

    /**
     * Helper to declare string parameters
     * @param name
     * @param value_default
     * @param min
     * @param max
     * @param step
     * @param description
     */
    template <typename T>
    void declare_default_parameter(
        const std::string &name,
        const T &value_default,
        const std::string &description)
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = description;
        n_.declare_parameter<T>(name, value_default, descriptor);
    }

    rclcpp::Node &n_;                                     /// reference to parent node
    rclcpp::TimerBase::SharedPtr timer_update_parameter_; /// timer to check regularly for parameter changes
    unsigned int parameter_update_cycle_;                 /// parameter defining the update time on check for parameter changes
};
/// some useful prototypes
using ParticleFilterNodeParameterPtr = std::shared_ptr<ParticleFilterNodeParameter>;
using ParticleFilterNodeParameterConstPtr = std::shared_ptr<ParticleFilterNodeParameter const>;

#endif // MR_PF_PKG__PARTICLE_FILTER_NODE_PARAMETER_HPP_
