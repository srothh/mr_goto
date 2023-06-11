#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_ekf/ekf_visualization.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

EKFVisualization::EKFVisualization()
    : EKF()
{
}

bool EKFVisualization::init(EKFParameter *param)
{
    param_ = (EKFVizParameter *)param;
    return EKF::init(param);
}

void EKFVisualization::prediction(const tuw::Command2DConstPtr u, double dt)
{
    this->draw();
    EKF::prediction(u, dt);
}

void EKFVisualization::callback_mouse(int event, int x, int y, int, void *param)
{
    EKFVisualization *node = reinterpret_cast<EKFVisualization *>(param);
    node->callback_mouse(event, x, y);
}

void EKFVisualization::callback_mouse(int event, int x, int y)
{

    static Point2D point;
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        point.set(figure_map_->m2w(Point2D(x, y)));
    }
    if (event == cv::EVENT_LBUTTONUP)
    {
        /**
         * @ToDo create a init pose from mouse click
         * similar to the particle filter
         **/
        /* your code start */
        reset_ = Reset::INTI_POSE;
        /* your code end   */
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

void EKFVisualization::draw(int delay)
{
    if (!figure_map_)
    {
        figure_map_ = make_shared<tuw::Figure>(param_->name);
        figure_map_->setLabel("x=%4.1f", "y=%4.1f");
        figure_map_->init(param_->map_cols, param_->map_rows,
                          param_->map_min_x, param_->map_max_x,
                          param_->map_min_y, param_->map_max_y,
                          param_->map_rotation + M_PI,
                          param_->map_grid_x, param_->map_grid_y, param_->map_file);
        cv::namedWindow(figure_map_->title(), 1);
        cv::moveWindow(figure_map_->title(), 20, 20);
    }
    figure_map_->clear();
    if (ground_truth_)
    {
        figure_map_->symbol(*ground_truth_, 0.2, Figure::cyan, 1);
    }
    if (estimated_pose_)
    {
        figure_map_->symbol(*estimated_pose_, 0.2, Figure::green, 1);
        draw_measurement(*estimated_pose_);
        draw_covariance();
    }

    draw_hspace();
    /**
     * @ToDo write your name into the view
     **/
    /* your code start */
    cv::putText(figure_map_->view(), "Rita Schrabauer", cv::Point(10, figure_map_->height() - 10), cv::FONT_HERSHEY_PLAIN, 1., cv::Scalar(64, 64, 64), 1, cv::LINE_AA);
    /* your code end   */
    cv::imshow(figure_map_->title(), figure_map_->view());
    cv::setMouseCallback(figure_map_->title(), callback_mouse, this);
    cv::waitKey(delay);
}

void EKFVisualization::draw_covariance()
{
    /**
     * @ToDo visualize the pose covariance with an ellipse which covers 95 %
     * Compute and plot the pose covariance in x and y direction
     * take the pose covariance P and create a 2x2 matrix out of the x,y components
     * transform the matrix into the plot E = Mw2m*P(0:1,0:1)*Mw2m'
     * use the opencv to compute eigenvalues and eigen vectors to compute the size and orientation of the ellipse
     * After the prediction, the ellipse of the covariance should be plotted here
     * Eigenvalues are used to visualize the ellipse with opencv ellipse
     * atan2 is needed to always return the angle from the x-axis and eliminate ambiguity (as opposed to atan)
     **/
    /* your code start */
    (void) x;
    (void) P;
    cv::Matx22d help;
    help(0, 0) = P(0, 0);
    help(0, 1) = P(0, 1);
    help(1, 0) = P(1, 0);
    help(1, 1) = P(1, 1);

    cv::Matx22d Mw2m (figure_map_->Mw2m()(0, 0), figure_map_->Mw2m()(0, 1), figure_map_->Mw2m()(1, 0), figure_map_->Mw2m()(1,1));
    cv::Matx33d Mw2d_whole_inv = figure_map_->Mw2m().inv();
    cv::Matx22d Mw2d_inv (Mw2d_whole_inv(0, 0), Mw2d_whole_inv(0, 1), Mw2d_whole_inv(1, 0), Mw2d_whole_inv(1, 1));
    
    cv::Matx<double, 2, 2> E = Mw2m*help*Mw2m.t();  /// must be changed
    cv::Mat_<double> eigval, eigvec; //cv::Mat_<double>
    cv::eigen ( E, eigval, eigvec );
    cv::Vec3d robot = figure_map_->Mw2m()*cv::Vec3d(x[0], x[1], 1.);
    cv::Point point;
    point.x = robot[0];
    point.y = robot[1];
    std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
    double chi_2_05 = 5.9991; //wie krieg ich den richtigen Wert?
    //std::cout << "eigval 0: " << eigval(0) << ", eigval 1:" << eigval(1) << std::endl;
    double width = 2*std::sqrt(abs(eigval(0))*chi_2_05);
    double height = 2*std::sqrt(abs(eigval(1))*chi_2_05);
    double theta = atan2(eigvec(0), eigvec(1))*180/M_PI;
    cv::RotatedRect ellipse (point, cv::Size ( width,height), theta); /// must be changed
    cv::ellipse ( figure_map_->view(),ellipse, Figure::magenta, 1, cv::LINE_AA );
    /* your code end   */
}
void EKFVisualization::draw_measurement(const Pose2D &pose_vehicle)
{
    cv::Scalar color = Figure::orange;
    /**
     * @ToDo drawing of the known map_linesegments_
     * visualize the line map (future map) stored in map_linesegments_
     **/
    /* your code start */
    for (int i = 0; i < map_linesegments_.size(); i++) {
        //cv::line(figure_map_, map_linesegments_.p0(), map_linesegments_);
        figure_map_->line(map_linesegments_.at(i).p0(), map_linesegments_.at(i).p1(), color);
        cv::Scalar color = Figure::orange;
        Point2D mitte;
        mitte.set_x((map_linesegments_.at(i).p0().get_x() + map_linesegments_.at(i).p1().get_x())/2);
        mitte.set_y((map_linesegments_.at(i).p0().get_y() + map_linesegments_.at(i).p1().get_y())/2);
        figure_map_->putText(std::to_string(i), mitte, cv::FONT_HERSHEY_PLAIN, 0.6, color);
    }
    /* your code end   */

    if (measurement_laser_scan_)
    {
        cv::Scalar color = Figure::orange;
        /**
         * @ToDo visualize the laser scan stored in measurement_laser_scan_
         **/
        /* your code start */
        tuw::Points2D laser_points = measurement_laser_scan_->data;
        tuw::Tf2D M = pose_vehicle.tf()*measurement_laser_scan_->tf;
        for(int i = 0; i < laser_points.size(); i++) {
            tuw::Point2D laser_point = M*laser_points.at(i);
            figure_map_->symbol(laser_point, color);
        }

        //(void) pose_vehicle;
        //(void) color;
        /* your code end   */
    }
    if (measurement_linesegments_)
    {

        cv::Scalar color_measurement = Figure::red;
        cv::Scalar color_match = Figure::blue;
        /**
         * @ToDo visualize the line segments stored in measurement_linesegments_
         * and the line matching stored in measurement_match_ with the line map.
         * We suggest to implement first the line drawing and after
         * you implemented EKF::data_association to enhance the drawing with
         * by visualization of the matching.
         **/
        /* your code start */
        //draw measurement
        tuw::LineSegments2D linesegments = measurement_linesegments_->data;
        tuw::Tf2D M = pose_vehicle.tf()*measurement_laser_scan_->tf;
        for(int i = 0; i < linesegments.size(); i++) {
            tuw::LineSegment2D line = linesegments.at(i);
            Point2D p0 = M*line.p0();
            Point2D p1 = M*line.p1();
            figure_map_->line(p0, p1, color_measurement);
            if (!measurement_match_.empty() && measurement_match_[i] >= 0) {
                LineSegment2D match_line = map_linesegments_[measurement_match_[i]];
                Point2D p0 = M*line.p0();
                Point2D p1 = M*line.p1();
                figure_map_->line(p0, p1, color_match);
            }
        }
        //draw match (muss ich mir noch durchüberlegen)
        //tuw::LineSegments2D predicted_linesegments = predicted_linesegments_;

        /*for(int i = 0; i < measurement_match_.size(); i++) {
            int idx = measurement_match_[i];
            tuw::LineSegment2D line = predicted_linesegments_.at(idx);
            
        }*/
        
        //(void) pose_vehicle;
        //(void) color_measurement;
        //(void) color_match;
        /* your code end   */
    }
}

void EKFVisualization::draw_hspace()
{

    if (!figure_hspace_)
    {
        /// Init hspace figure
        figure_hspace_ = std::make_shared<Figure>(param_->name + std::string(" Hough Space"));
        figure_hspace_->setLabel("alpha=%4.2f", "rho=%4.2f");
        figure_hspace_->init(param_->hough_space_pixel_alpha, param_->hough_space_pixel_rho,
                             -M_PI * 1.1, +M_PI * 1.1,
                             0, param_->hough_space_meter_rho,
                             M_PI,
                             1, M_PI / 4);
        cv::namedWindow(figure_hspace_->title(), 1);
        cv::moveWindow(figure_hspace_->title(), 640, 20);
    }

    figure_hspace_->clear();
    cv::Rect rectSpace(0, 0, figure_hspace_->view().cols, figure_hspace_->view().rows);
    if (measurement_laser_scan_)
    {
        for (unsigned int i = 0; i < measurement_laser_scan_->data.size(); i++)
        {
            Point2D p0 = measurement_laser_scan_->data[i];
            for (double alpha = figure_hspace_->min_x(); alpha < figure_hspace_->max_x(); alpha += 1.0 / figure_hspace_->scale_x())
            {
                /**
                 * @ToDo laser beams in hough space
                 * draw a wave with angle = [-pi...pi], r = x*cos(angle) + y *sin(angle) for every laser point [x,y].
                 * The function Line2D::toPolar() can be used to transform a line into polar coordinates
                 * Plot the lines (measurements) in hough coordinates.
                 * Each laser scan point that corresponds to a line is transformed into hough coordinates here.
                 * The intersection of all of them yields the resulting value for the hough representation of each line
                 **/
                /* your code start */
                //double r = p0.get_x()*cos(alpha) + p0.get_y()*sin(alpha);
                Point2D point;
                point.set_x(p0.get_x() + cos(alpha));
                point.set_y(p0.get_y() + sin(alpha));
                Line2D line;
                line.set(p0, point);
                tuw::Polar2D polar = line.toPolar();
                cv::Point point_h = (figure_hspace_->w2m(polar)).cv();
                //jetzt noch plotten
                //figure_hspace_->circle(polar, 1, figure_hspace_->yellow);
                figure_hspace_->view().at<cv::Vec3b>(point_h) -= cv::Vec3b(50, 10, 10);
                /* your code end   */
            }
        }
    }
    cv::Scalar color;
    if (measurement_linesegments_)
    {
        /**
         * @ToDo measurement in hough space
         * Draw the measurement (measurement_linesegments_) in the hough space as a circle or dot.
         * You should also write the index of each line segment next to it for debugging
         */
        color = Figure::red;
        /* your code start */
        tuw::LineSegments2D line_segments = measurement_linesegments_->data;
        for (int i = 0; i < line_segments.size(); i++) {
            tuw::Line2D line = line_segments.at(i).line();
            tuw::Polar2D polar_coord = line.toPolar();
            figure_hspace_->circle(polar_coord, 3, color);
            figure_hspace_->putText(std::to_string(i), polar_coord, cv::FONT_HERSHEY_PLAIN, 0.6, color);
        }
        /* your code end   */
    }
    /**
     * @ToDo prediction measurement in hough space
     * Draw the measurement prediction (predicted_linesegments_) in the hough space as a circle or dot.
     * You should also write the index of each line segment next to it for debugging
     */
    color = Figure::orange;
    /* your code start */
    for (int i = 0; i < predicted_linesegments_.size(); i++) {
        tuw::Line2D line = predicted_linesegments_.at(i).line();
        tuw::Polar2D polar_coord = line.toPolar();
        figure_hspace_->circle(polar_coord, 1, color);
        figure_hspace_->putText(std::to_string(i), polar_coord, cv::FONT_HERSHEY_PLAIN, 0.6, color);
    }
     /* your code end   */
    cv::imshow(figure_hspace_->title(), figure_hspace_->view());
}
