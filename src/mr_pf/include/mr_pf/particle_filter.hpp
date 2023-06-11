#ifndef MR_PF_PKG__PARTICLE_FILTER_HPP_
#define MR_PF_PKG__PARTICLE_FILTER_HPP_
#include <memory>
#include <random>
#include <tuw_geometry/tuw_geometry.hpp>
#include <mr_pf/particle_filter_parameter.hpp>
#include <chrono>

namespace mr
{
    using namespace std;
    /// some useful prototypes
    class ParticleFilter;
    using ParticleFilterPtr = std::shared_ptr<ParticleFilter>;
    using ParticleFilterConstPtr = std::shared_ptr<ParticleFilter const>;

    /**
     * Particle filter class without visualization
     */
    class ParticleFilter
    {
    public:
        /**
         * Constructor
         */
        ParticleFilter();

        /**
         * @brief  initializes the filter
         * The functions needs to be virutal to allow the visualization call to overwrite it
         * @param param
         * @return true on error
         **/
        virtual bool init(ParticleFilterParameterPtr param);

        /**
         * motion update on all samples
         * @param u last motion command executed
         * @param dt seconds since last call
         */
        virtual void update(const tuw::Command2DConstPtr u, double dt);

        /**
         * resamples samples based on there weights
         * @param dt seconds since last call used to scale noise if needed
         */
        void resample(double dt);

        /**
         * sets ground truth pose if know like on a simulation
         * @param p pose
         */
        void set_ground_truth(const tuw::Pose2D &p);

        /**
         * sets a pose for inalialization
         * @see ParticleFilterParameter::reset_
         * @param p pose
         */
        void set_init_pose(const tuw::Pose2D &p);

        /**
         * compute sample weights
         * @param z_s measurement in sensor frame
         * @param pose_sensor pose sensor in base frame
         */
        void compute_weights(const std::vector<std::pair<tuw::Point2D, tuw::Polar2D>> &z_s, const tuw::Pose2D &pose_sensor);

        /**
         * compute pose estimate
         * @return pose or null
         */
        tuw::Pose2DPtr compute_estimated_pose();

    protected:
        /**
         * computes the likelihood field based on a map matrix
         */
        void compute_likelihood_field();

        /**
         * up- or down sizes the number of samples
         */
        void generate_samples();

        /**
         * resets a samples based on reset_
         * @see ParticleFilter::Reset
         */
        void reset_samples();

        enum Reset
        {
            OFF,
            UNIFORM,
            GROUND_TRUTH,
            INTI_POSE
        };
        tuw::WorldScopedMaps map_header_;           /// map header
        tuw::Pose2DPtr ground_truth_;               /// ground truth vehicle location if available
        tuw::Pose2DPtr init_pose_;                  /// init pose if available
        tuw::Pose2DPtr estimated_pose_;             /// pose estimated by the filter
        std::vector<tuw::Point2D> z_r_;             /// laser measurements in sensor frame
        tuw::Tf2D tf_base_sensor_;                  /// transformation base sensor
        cv::Mat_<uint8_t> map_;                     /// obstacle map image as opencv matrix
        cv::Mat_<float> distance_field_pixel_;      /// distance field in pixels
        cv::Mat_<float> distance_field_;            /// distance field in meters
        cv::Mat_<float> likelihood_field_;          /// computed likelihood field
        ParticleFilterParameterPtr param_;          /// Parameter used
        std::vector<tuw::SamplePose2DPtr> samples_; /// particle samples
        double samples_weight_max_;                 /// current heights scoring sample weight
        Reset reset_;                               /// resets the filter on true onces unless param_::continues_reset is true
        std::chrono::microseconds processing_time_update_;
        std::chrono::microseconds processing_time_resample_;
        std::chrono::microseconds processing_time_compute_weights_;

    private:
        std::random_device rd_;                                /// random number device
        std::mt19937 generator_;                               /// random number generator
        std::uniform_real_distribution<double> uniform_x_;     /// uniform distribution used for generate a random x on the map
        std::uniform_real_distribution<double> uniform_y_;     /// uniform distribution used for generate a random y on the map
        std::uniform_real_distribution<double> uniform_theta_; /// uniform distribution used for generate a random angle
        std::uniform_real_distribution<double> uniform_;       /// uniform distribution zero to 1
        std::normal_distribution<double> normal_distribution_; /// normal distribution for generic use

        /**
         * sets uniform pose on a given sample inside the defined map
         * @param des sample to set
         * @return reference to sample des
         */
        tuw::SamplePose2DPtr &sample_uniform(tuw::SamplePose2DPtr &des);
        /**
         * sets a pose with gaussian noise on a sample
         * @param des sample to set
         * @param src source pose
         * @param sigma_position
         * @param sigma_orientation
         * @return reference to sample des
         */
        tuw::SamplePose2DPtr &sample_normal(tuw::SamplePose2DPtr &des, tuw::Pose2D &src, double sigma_position, double sigma_orientation);
    };
}

#endif // MR_PF_PKG__PARTICLE_FILTER_HPP_