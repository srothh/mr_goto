#ifndef MR_POSE_ESTIMATION_PKG__EKF_PARAMETER_HPP_
#define MR_POSE_ESTIMATION_PKG__EKF_PARAMETER_HPP_
#include <memory>
#include <random>
#include <tuw_geometry/tuw_geometry.hpp>
#include <chrono>

namespace mr
{
    /**
     * Class to hold the particle filter parameters
     */
    class EKFParameter
    {
    public:
        double forward_prediction_time;
        double init_sigma_location;
        double init_sigma_orientation;
        bool enable_prediction;
        bool enable_data_association;
        bool enable_correction;
        double alpha_1, alpha_2, alpha_3, alpha_4;
        double measurement_noise_rho, measurement_noise_alpha;
        double hough_space_pixel_rho;
        double hough_space_pixel_alpha;
        double hough_space_meter_rho;
        double data_association_line_rho;
        double data_association_line_alpha;
        double data_association_distance_to_endpoints;
        std::string map_linesegments_file; /// path to feature map used
    };
    using EKFParameterPtr = std::shared_ptr<EKFParameter>;
    using EKFParameterConstPtr = std::shared_ptr<EKFParameter const>;
}

#endif // MR_POSE_ESTIMATION_PKG__EKF_PARAMETER_HPP_
