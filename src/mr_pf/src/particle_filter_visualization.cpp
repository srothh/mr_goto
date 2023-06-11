#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_pf/particle_filter_visualization.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

ParticleFilterVisualization::ParticleFilterVisualization()
    : ParticleFilter()
{
}

bool ParticleFilterVisualization::init(ParticleFilterParameterPtr config)
{
    figure_ = make_shared<tuw::Figure>(config->name);
    figure_->init(config->map_cols, config->map_rows,
                  config->map_min_x, config->map_max_x,
                  config->map_min_y, config->map_max_y,
                  config->map_rotation + M_PI,
                  config->map_grid_x, config->map_grid_y, config->map_file);

    cv::namedWindow(figure_->title(), 1);
    return ParticleFilter::init(config);
}

void ParticleFilterVisualization::update(const tuw::Command2DConstPtr u, double dt)
{

    plot_likelihood_field();
    draw(1, dt);
    ParticleFilter::update(u, dt);
}

void ParticleFilterVisualization::callback_mouse(int event, int x, int y, int, void *param)
{
    ParticleFilterVisualization *node = reinterpret_cast<ParticleFilterVisualization *>(param);
    node->callback_mouse(event, x, y);
}

void ParticleFilterVisualization::callback_mouse(int event, int x, int y)
{

    /**
     * @ToDo Sensor & Motion Model
     * Reset the filter with a BUTTON to a defined pose
     * the position should be defined by the click (down) and the orienation by the button release (up)
     **/
    if (param_->level > ParticleFilterParameter::PLOT_LASER_MEASUREMENT)
    {
    }
    else
    {
        /// @node your code
        //(void)x; /// to silence a warning about unused variables
        //(void)y; /// to silence a warning about unused variables
        
        if (event == cv::EVENT_LBUTTONDOWN) {
            tuw::Pose2D p;
            p.set_x(x);
            p.set_y(y);
            ParticleFilter::set_init_pose(p);
        }
        if (event == cv::EVENT_LBUTTONUP) {
            tuw::Pose2D p;
            tuw::Point2D point;
            p.set_x(init_pose_->get_x());
            p.set_y(init_pose_->get_y());
            /*double map_grid_x = 558;
            double map_real_x = 9; //min = -9
            double map_grid_y = 558;
            double map_real_y = 9; //min = -9
            double real_x = (x - p.get_x())/map_grid_x *18 - 9;
            double real_y = (x - p.get_x())/map_grid_y *18 - 9;
            point.set_x(real_x); //x-p.get_x()
            point.set_y(real_y); //y-p.get_y()*/
            point.set_x(x-p.get_x());
            point.set_y(y-p.get_y());
            p.set_theta(point.angle());

            //std::cout << "Maus: x: " << p.get_x() << ", y: " << p.get_y() << ", Theta: " << p.get_theta() << std::endl;
            ParticleFilter::set_init_pose(p);
            reset_ = Reset::INTI_POSE;
        }
        
    }
    if (event == cv::EVENT_RBUTTONUP)
    {
        reset_ = Reset::GROUND_TRUTH;
    }
    if (event == cv::EVENT_MBUTTONUP)
    {
        reset_ = Reset::UNIFORM;
    }
}

void ParticleFilterVisualization::draw(int delay, double dt)
{
    figure_->clear();
    this->plot_laser_measurement_ground_truth();
    this->plot_samples();
    char txt[0xFF];
    sprintf(txt, "duration = %3.2f, nr of samples %zu", dt, samples_.size());
    cv::putText(figure_->view(), txt, cv::Point(10, figure_->height() - 15), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
    cv::putText(figure_->view(), txt, cv::Point(10, figure_->height() - 15), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
    int spacing = 8;
    sprintf(txt, "%05zu us update", processing_time_update_.count());
    cv::putText(figure_->view(), txt, cv::Point(10, spacing * 1), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
    sprintf(txt, "%05zu us resample", processing_time_resample_.count());
    cv::putText(figure_->view(), txt, cv::Point(10, spacing * 2), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
    sprintf(txt, "%05zu us compute weight", processing_time_compute_weights_.count());
    cv::putText(figure_->view(), txt, cv::Point(10, spacing * 3), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);

    cv::imshow(figure_->title(), figure_->view());
    cv::setMouseCallback(figure_->title(), callback_mouse, this);
    cv::waitKey(delay);
}

void ParticleFilterVisualization::plot_laser_measurement_ground_truth()
{
    if (ground_truth_)
    {
        figure_->symbol(*ground_truth_, 0.2, Figure::cyan, 1);
        plot_laser_measurement(*ground_truth_, z_r_);
    }
}

void ParticleFilterVisualization::plot_laser_measurement(const Pose2D &pose_vehicle, const vector<Point2D> &z)
{
    /**
     * @ToDo Sensor Model
     * Plot the laser measurement around a given robot pose pose.
     * Hint:
     *  Figure::symbol(Point2D)
     *  tf_base_sensor_
     *  pose_vehicle
     **/
    if (param_->level > ParticleFilterParameter::PLOT_LASER_MEASUREMENT)
    {
    }
    else
    {
        /// @note Your code here

        tuw::Tf2D T = tf_base_sensor_*pose_vehicle.tf();
        
        for (size_t i = 0; i < z.size(); i++)
        {
            tuw::Point2D point;
            //point = T*z[i].vector();
            figure_->symbol(point, Figure::red);
        }

        // Dummy code below should be removed.
        //(void)pose_vehicle; /// to silence a warning about unused variables
        //(void)z;            /// to silence a warning about unused variables
    }
}

/**
 * @ToDo Sensor Model
 **/
void ParticleFilterVisualization::plot_likelihood_field()
{
    /**
     * @ToDo Sensor Model
     * Draw the likelihood field onto the background
     * Hint:
     * * cv::minMaxLoc
     **/
    if (param_->level > ParticleFilterParameter::PLOT_LIKELIHOOD_FIELD)
    {
    }
    else
    {
        /// @note your code here

        double min, max;
        cv::minMaxLoc(likelihood_field_.reshape(1), &min, &max, NULL, NULL);
        //likelihood_field_
        //normal_distribution

        cv::Mat background = figure_->background();
        //cv::Mat newBackground = figure_->;
        cv::Mat view = figure_->view();

        for (int x = 0; x < likelihood_field_.rows; x++)
        {
           for (int y = 0; y < likelihood_field_.cols; y++)
           {
                cv::Vec3b &px = background.at<cv::Vec3b>(x, y);
                //px[0] = 255;
                //px[1] = 255;
                //background.at<cv::Vec3b>(x, y) = px;

                float likelihood = likelihood_field_.at<float>(x, y);
                cv::Scalar color;

                if(likelihood > 0)
                    color = cv::Scalar(255*(1-likelihood), 255/*(1-likelihood)*/, 255/*(1-likelihood)*/);
                //else
                //    color = cv::Scalar(255, 255, 255);

                if(px[0] == 255 && px[1] == 255 && px[2] == 255)
                    cv::circle(background, cv::Point2d(y, x), 1, color);
           }
        }
        //cv::imshow(figure_->title(), view);
        //figure_->clear();

    /*    char txt[0xFF];
    sprintf(txt, "duration = 3.2f, nr of samples zu");
    cv::putText(figure_->view(), txt, cv::Point(figure_->width() - 10, figure_->height() - 15), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
     cv::imshow(figure_->title(), figure_->view());   */
        
        

        //cv::line(0, 1, Figure::blue, 1, 16);

        /*for(int x = 0; x < figure_->background().rows; x++)
        {
            for(int y = 0; y < figure_->background().cols; y++) {
                figure_->background().at<cv::Vec3b>(x, y)[1] = 255;
            }
        }*/
    }
}

/**
 * @ToDo Sensor Model
 *
 *
 **/
void ParticleFilterVisualization::plot_samples()
{
    /**
     * @ToDo Sensor Model
     * draw all particles with weights
     * Hints:
     * @see tuw::Figure::symbol()
     **/
    if (param_->level > ParticleFilterParameter::PLOT_SAMPLES)
    {
    }
    else
    {
        /// @note your code here

        double scale = 255.0 / samples_weight_max_;
        for (auto it = std::rbegin(samples_); it != std::rend(samples_); ++it) {
            float scaling = it->get()->weight();
            cv::Scalar color = cv::Scalar(scale*scaling, scale*(samples_weight_max_-scaling), 0);
            figure_->symbol(Point2D(it->get()->position()), 0.2, color, 1);
            //std::cout << "x: " << it->get()->get_x() << ", y: " << it->get()->get_y() << std::endl; 
        }
    }
}
