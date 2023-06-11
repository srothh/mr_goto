#ifndef MR_POSE_ESTIMATION_PKG__EKF_VIZ_PARAMETER_HPP_
#define MR_POSE_ESTIMATION_PKG__EKF_VIZ_PARAMETER_HPP_
#include <rclcpp/rclcpp.hpp>
#include <mr_ekf/ekf_parameter.hpp>

namespace mr
{
    /**
     * class to handle particle filter parameter using ros parameter callbacks
     */
    class EKFVizParameter : public EKFParameter
    {
    public:
        int map_cols;                  /// map size used width [pixel]
        int map_rows;                  /// map size used height [pixel]
        double map_min_x, map_max_x;   /// map shown in x [m]
        double map_min_y, map_max_y;   /// map shown in y [m]
        double map_rotation;           /// map shown rotation
        double map_grid_y, map_grid_x; /// map shown grid resolution
        std::string map_file;          /// path to map for debugging used
        std::string name;              /// window name used if the data is visualized
    };
    /// some useful prototypes
    using EKFVizParameterPtr = std::shared_ptr<EKFVizParameter>;
    using EKFVizParameterConstPtr = std::shared_ptr<EKFVizParameter const>;
}
#endif // MR_POSE_ESTIMATION_PKG__EKF_VIZ_PARAMETER_HPP_
