#ifndef MR_POSE_ESTIMATION_PKG__EKF_HPP_
#define MR_POSE_ESTIMATION_PKG__EKF_HPP_
#include <memory>
#include <random>
#include <tuw_geometry/tuw_geometry.hpp>
#include <mr_ekf/ekf_parameter.hpp>
#include <chrono>

namespace mr
{
    using namespace std;
    /// some useful prototypes
    class EKF;
    using EKFPtr = std::shared_ptr<EKF>;
    using EKFConstPtr = std::shared_ptr<EKF const>;

    /**
     * Particle filter class without visualization
     */
    class EKF
    {
    public:
        /**
         * Constructor
         */
        EKF();

        /**
         * @brief  initializes the filter
         * The functions needs to be virutal to allow the visualization call to overwrite it
         * @param param
         * @return true on error
         **/
        virtual bool init(EKFParameter *param);

        /**
         * @brief  forward motion
         * @param u command
         * @param dt duration of command execution
         **/
        virtual void prediction(const tuw::Command2DConstPtr u, double dt);

        /**
         * @brief pose correction based on line segments
         * @param z measurement
         **/
        virtual void correction(tuw::StampedDataPtr<tuw::LineSegments2D> z);

        /**
         * @brief pose correction based on markers
         * @param z measurement
         * @note dummy for future use
         **/
        virtual void correction(tuw::StampedDataPtr<tuw::Points2D> z);

        /**
         * @brief called before every interation
         * @note if the reset_ variale is set it will reset the system accordinly
         **/
        void reset();

        /**
         * sets ground truth pose if know like on a simulation
         * @param p pose
         */
        void set_ground_truth(const tuw::Pose2D &p);

        /**
         * sets a pose for inalialization
         * @see EKFParameter::reset_
         * @param p pose
         */
        void set_init_pose(const tuw::Pose2D &p);

        /**
         * compute pose estimate
         * @return pose or null
         */
        tuw::Pose2DPtr compute_estimated_pose();

        /**
         * reads the line sements for the map file
         */
        virtual void load_map();

    private:
        void data_association();

    protected:
        enum Reset
        {
            OFF,
            UNIFORM,
            GROUND_TRUTH,
            INTI_POSE,
            CENTER,
        };
        Reset reset_;
        EKFParameter *param_;                                               /// Parameter used
        tuw::Pose2DPtr ground_truth_;                                       /// ground truth vehicle location if available
        tuw::Pose2DPtr init_pose_;                                          /// init pose if available
        tuw::Pose2DPtr estimated_pose_;                                     /// pose estimated by the filter
        std::vector<tuw::Point2D> z_r_;                                     /// laser measurements in sensor frame
        tuw::Tf2D tf_base_sensor_;                                          /// transformation base sensor
        tuw::WorldScopedMaps hspace_header_;                                /// map header
        std::vector<int> measurement_match_;                                /// index vector of successful matched measurements with predicted measurements
        tuw::Pose2DPtr pose_predicted_;                                     /// predicted pose derived form xp
        cv::Vec<double, 3> x;                                               /// state x
        cv::Matx<double, 3, 3> P;                                           /// covariance for x
        cv::Vec<double, 3> xp;                                              /// prediction x
        cv::Matx<double, 3, 3> Pp;                                          /// predicted covariance for x
        cv::Vec<double, 3> xc;                                              /// corrected x
        cv::Matx<double, 3, 3> Pc;                                          /// corrected covariance for x
        cv::Matx<double, 3, 3> G;                                           /// Motion model derivation to x
        cv::Matx<double, 3, 3> R;                                           /// V*M*V'   --> Thrun
        cv::Matx<double, 3, 2> V;                                           /// Motion model
        cv::Matx<double, 2, 2> M;                                           /// Motion covariance
        cv::Matx<double, 2, 2> Q;                                           /// Detection covariance
        tuw::LineSegments2D map_linesegments_;                              /// known line segments in world coordinates
        tuw::LineSegments2D predicted_linesegments_;                        /// known line segments in sensor coordinates
        tuw::StampedDataPtr<tuw::LineSegments2D> measurement_linesegments_; /// detected line segments in sensor coordinates
        tuw::StampedDataPtr<tuw::Points2D> measurement_laser_scan_;         /// last received laser scan
    };
}

#endif // MR_POSE_ESTIMATION_PKG__EKF_HPP_
