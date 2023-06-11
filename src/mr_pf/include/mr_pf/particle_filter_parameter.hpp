#ifndef MR_PF_PKG__PARTICLE_FILTER_PARAMETER_HPP_
#define MR_PF_PKG__PARTICLE_FILTER_PARAMETER_HPP_
#include <memory>
#include <random>
#include <tuw_geometry/tuw_geometry.hpp>
#include <chrono>

namespace mr
{
    /// some useful prototypes
    class ParticleFilterParameter;
    using ParticleFilterParameterPtr = std::shared_ptr<ParticleFilterParameter>;
    using ParticleFilterParameterConstPtr = std::shared_ptr<ParticleFilterParameter const>;

    /**
     * Class to hold the particle filter parameters
     */
    class ParticleFilterParameter
    {
    public:
        enum ResampleStrategy
        {
            LOW_VARIANCE,
            KEEP_BEST
        };
        enum Level : unsigned int
        {
            CHECK_MAP_FILE_EXISTS = 0,
            CATCH_LASER_TRANSFORMATION = 1,
            PLOT_LASER_MEASUREMENT = 2,
            PLOT_LIKELIHOOD_FIELD = 3,
            COMPUTE_LIKELIHOOD_FIELD = 4,
            PLOT_SAMPLES = 5,
            COMPUTE_SAMPLES_WEIGHT = 6,
            MOTION_UPDATE = 7,
            RESAMPLING = 8,
            USE_RANDOM_BEAMS = 9,
            INT_CLICK = 10
        };
        Level level;                                           /// implementation level for students
        std::string name;                                      /// window name used if the data is visualized
        unsigned int nr_of_samples;                            /// number of samples used
        int map_cols;                                          /// map size used width [pixel]
        int map_rows;                                          /// map size used height [pixel]
        double map_min_x, map_max_x;                           /// map shown in x [m]
        double map_min_y, map_max_y;                           /// map shown in y [m]
        double map_rotation;                                   /// map shown rotation
        double map_grid_y, map_grid_x;                         /// map shown grid resolution
        std::string map_file;                                  /// path to map used
        double sigma_hit;                                      /// senor model parameters
        double z_hit;                                          /// sensor model parameters
        double z_short;                                        /// senor model parameters
        double z_rand;                                         /// senor model parameters
        double z_max;                                          /// senor model parameters
        double rate_of_beams_used;                             /// number of beams use
        bool random_beams_used;                                /// on true random beams are used
        bool disable_update;                                   /// disable update
        double duration_offset;                                /// duration offset / offset for forward prediction
        bool continues_reset;                                  /// on true reset data after each interation
        double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6; /// Motion Model parameter
        double resample_rate;                                  /// resample rate
        double resample_noise_position;                        /// general position noise on new particles
        double resample_noise_orientation;                     /// general orientation noise on new particles
        ResampleStrategy resample_strategy;                    /// resample strategy
    };
}

#endif // MR_PF_PKG__PARTICLE_FILTER_PARAMETER_HPP_
