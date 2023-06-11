#ifndef MR_POSE_ESTIMATION_PKG__EKF_VISUALIZATION_HPP_
#define MR_POSE_ESTIMATION_PKG__EKF_VISUALIZATION_HPP_
#include <memory>
#include <random>
#include <tuw_geometry/tuw_geometry.hpp>
#include <mr_ekf/ekf.hpp>
#include <mr_ekf/ekf_visualization_parameter.hpp>
#include <chrono>

namespace mr
{
    /**
     * Class for to visualize sensor readings with an OpenCV window
     */
    class EKFVisualization : public EKF
    {
    public:
        /**
         * Constructor
         */
        EKFVisualization();

        /**
         * initializes the filter and the debug visualization
         * @param param parameters used
         * @return true on error
         */
        bool init(EKFParameter* param);

        /**
         * calles a draw function and then forward motion prediction
         * @param u command
         * @param dt duration of command execution
         **/
        void prediction(const tuw::Command2DConstPtr u, double dt);

        /// Callback function for opencv window (static version)
        static void callback_mouse(int event, int x, int y, int flags, void *param);
        
    private:
        /// Callback function for opencv window (member version)
        void callback_mouse(int event, int x, int y);
        /**
         * draws an open cv image and calls
         * - draw_measurement();
         * - draw_covariance();
         * - draw_hspace();
         * @param delay for the opencv draw function [ms]
         */
        void draw(int delay = 1);
        void draw_measurement(const tuw::Pose2D &pose_vehicle);
        void draw_covariance();
        void draw_hspace();

        EKFVizParameter *param_;  /// Parameter used
        tuw::FigurePtr figure_map_;   /// figure to plot debug information
        tuw::FigurePtr figure_hspace_;   /// figure to plot debug information
    };
}

#endif // MR_POSE_ESTIMATION_PKG__EKF_VISUALIZATION_HPP_
