#ifndef MR_PF_PKG__PARTICLE_FILTER_VISUALIZATION_HPP_
#define MR_PF_PKG__PARTICLE_FILTER_VISUALIZATION_HPP_
#include <memory>
#include <random>
#include <tuw_geometry/tuw_geometry.hpp>
#include <mr_pf/particle_filter.hpp>
#include <chrono>

namespace mr
{
    /**
     * Class for to visualize sensor readings with an OpenCV window
     */
    class ParticleFilterVisualization : public ParticleFilter
    {
    public:
        /**
         * Constructor
         */
        ParticleFilterVisualization();

        /**
         * initializes the particle filter and the debug visualization
         * @param param parameters used
         * @return true on error
         */
        bool init(ParticleFilterParameterPtr param);

        /**
         * calls update on the base class and draws the likelihood field
         * @param param parameters used
         * @return true on error
         */
        void update(const tuw::Command2DConstPtr u, double dt);

        /// Callback function for opencv window (static version)
        static void callback_mouse(int event, int x, int y, int flags, void *param);

    private:
        /// Callback function for opencv window (member version)
        void callback_mouse(int event, int x, int y);
        /**
         * draws an open cv image and calls
         * -  plot_laser_measurement_ground_truth();
         * -  plot_samples();
         */
        void draw(int delay, double dt);

        /**
         * plots laser measurement
         * on ground truth data it it exists
         */
        void plot_laser_measurement_ground_truth();
        /**
         * plots laser measurement
         * for a given vehicle pose
         * @param pose_vehicle
         * @param z laser measurement in sensor space
         */
        void plot_laser_measurement(const tuw::Pose2D &pose_vehicle, const std::vector<tuw::Point2D> &z);
        /**
         * plots likelihood field
         */
        void plot_likelihood_field();
        /**
         * plots samples
         */
        void plot_samples();
        tuw::FigurePtr figure_; /// figure to plot debug information
    };
}

#endif // MR_PF_PKG__PARTICLE_FILTER_VISUALIZATION_HPP_