#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_ekf/ekf.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

EKF::EKF()
{
}
bool EKF::init(EKFParameter *param)
{
    param_ = param;
    reset_ = Reset::GROUND_TRUTH;
    hspace_header_.init(param_->hough_space_pixel_alpha, param_->hough_space_pixel_rho,
                        -M_PI * 1.1, +M_PI * 1.1,
                        0, param_->hough_space_meter_rho,
                        M_PI);
    return false;
}

void EKF::set_ground_truth(const Pose2D &p)
{
    if (!ground_truth_)
        ground_truth_ = make_shared<Pose2D>(p);
    else
        *ground_truth_ = p;
    ground_truth_->recompute_cached_cos_sin();
}

void EKF::set_init_pose(const Pose2D &p)
{
    if (!init_pose_)
        init_pose_ = make_shared<Pose2D>(p);
    else
        *init_pose_ = p;
}

Pose2DPtr EKF::compute_estimated_pose()
{
    return estimated_pose_;
}

void EKF::prediction(const Command2DConstPtr u, double dt)
{
    reset();
    if (!estimated_pose_ || !u || fabs(dt) <= FLT_MIN)
        return;

    x = estimated_pose_->state_vector();
    if (param_->enable_prediction)
    {

        /**
         * @ToDo  pose prediction and covariance prediction
         * compute KalmanFilter::xp and KalmanFilter::Pp as predicted pose and Covariance
         * Computes the prediction step of the Kalman filter as in Thrun et. al.
         **/
        /* your code start */
        double theta = x[2];
        double vt = u->v();
        double wt = u->w();

        if (abs(wt) >= FLT_MIN)
        {
            // set G
            G(0, 0) = 1;
            G(0, 1) = 0;
            G(0, 2) = -vt / wt * cos(theta) + vt / wt * cos(theta + wt * dt);

            G(1, 0) = 0;
            G(1, 1) = 1;
            G(1, 2) = -vt / wt * sin(theta) + vt / wt * sin(theta + wt * dt);

            G(2, 0) = 0;
            G(2, 1) = 0;
            G(2, 2) = 1;

            // set V
            V(0, 0) = (-sin(theta) + sin(theta + wt * dt)) / wt;
            V(0, 1) = (vt * (sin(theta) - sin(theta + wt * dt))) / (wt * wt) + (vt * cos(theta + wt * dt) * dt) / wt;

            V(1, 0) = (cos(theta) - cos(theta + wt * dt)) / wt;
            V(1, 1) = (-vt * (cos(theta) - cos(theta + wt * dt))) / (wt * wt) + (vt * sin(theta + wt * dt) * dt) / wt;

            V(2, 0) = 0;
            V(2, 1) = dt;

            // set M
            M(0, 0) = param_->alpha_1 * vt * vt + param_->alpha_2 * wt * wt;
            M(0, 1) = 0;

            M(1, 0) = 0;
            M(1, 1) = param_->alpha_3 * vt * vt + param_->alpha_4 * wt * wt;

            // Hilfsmatrix
            cv::Vec3d help;
            help(0) = -vt / wt * sin(theta) + vt / wt * sin(theta + wt * dt);
            help(1) = vt / wt * cos(theta) - vt / wt * cos(theta + wt * dt);
            help(2) = wt * dt;

            // set xp
            xp = x + help;
            std::cout << "xp[0]: " << xp[0] << " xp[1]: " << xp[1] << " xp[2]: " << xp[2] << std::endl;

            // set Pp
            Pp = G * P * G.t() + V * M * V.t();
        }
        else
        {

            G(0, 0) = 1;
            G(0, 1) = 0;
            G(0, 2) = -dt * vt * sin(theta);

            G(1, 0) = 0;
            G(1, 1) = 1;
            G(1, 2) = dt * vt * cos(theta);

            G(2, 0) = 0;
            G(2, 1) = 0;
            G(2, 2) = 1;

            // set V
            V(0, 0) = dt * cos(theta);
            V(0, 1) = -1 / 2 * dt * dt * vt * sin(theta);

            V(1, 0) = dt * sin(theta);
            V(1, 1) = 1 / 2 * dt * dt * vt * cos(theta);

            V(2, 0) = 0;
            V(2, 1) = dt;

            // set M
            M(0, 0) = param_->alpha_1 * vt * vt + param_->alpha_2 * wt * wt;
            M(0, 1) = 0;

            M(1, 0) = 0;
            M(1, 1) = param_->alpha_3 * vt * vt + param_->alpha_4 * wt * wt;

            // Hilfsmatrix
            cv::Vec3d help;
            help(0) = dt * vt * cos(theta);
            help(1) = dt * vt * sin(theta);
            help(2) = wt * dt;

            // set xp
            xp = x + help;
            std::cout << "xp[0]: " << xp[0] << " xp[1]: " << xp[1] << " xp[2]: " << xp[2] << std::endl;

            // set Pp
            Pp = G * P * G.t() + V * M * V.t();
        }
    }
    else
    {
        xp = x;
        Pp = P;
    }
    if (!pose_predicted_)
        pose_predicted_ = make_shared<Pose2D>();
    pose_predicted_->set(xp);
}

void EKF::load_map()
{
    cv::FileStorage fs(param_->map_linesegments_file, cv::FileStorage::READ);
    cv::Mat_<double> l;
    fs["line segments"] >> l;
    map_linesegments_.resize(l.rows);
    for (size_t i = 0; i < map_linesegments_.size(); i++)
    {
        map_linesegments_[i].set(l(i, 0), l(i, 1), l(i, 2), l(i, 3));
    }
}
void EKF::reset()
{
    switch (reset_)
    {
    case Reset::UNIFORM:
        estimated_pose_ = make_shared<Pose2D>();
        P = cv::Matx33d(param_->init_sigma_location * param_->init_sigma_location, 0, 0,
                        0, param_->init_sigma_location * param_->init_sigma_location, 0,
                        0, 0, param_->init_sigma_orientation * param_->init_sigma_orientation);
        reset_ = Reset::OFF;
        break;
    case Reset::GROUND_TRUTH:
        if (ground_truth_)
        {
            estimated_pose_ = make_shared<Pose2D>(*ground_truth_);
            P = cv::Matx33d(param_->init_sigma_location * param_->init_sigma_location, 0, 0,
                            0, param_->init_sigma_location * param_->init_sigma_location, 0,
                            0, 0, param_->init_sigma_orientation * param_->init_sigma_orientation);
            reset_ = Reset::OFF;
        }
        break;
    case Reset::INTI_POSE:
        if (init_pose_)
        {
            estimated_pose_ = make_shared<Pose2D>(*init_pose_);
            P = cv::Matx33d(param_->init_sigma_location * param_->init_sigma_location, 0, 0,
                            0, param_->init_sigma_location * param_->init_sigma_location, 0,
                            0, 0, param_->init_sigma_orientation * param_->init_sigma_orientation);
            reset_ = Reset::OFF;
        }
        break;
    default:
        break;
    }
}

void EKF::correction(StampedDataPtr<LineSegments2D> z)
{
    measurement_linesegments_ = z;
    if (!pose_predicted_)
        return;
    data_association();

    xc = pose_predicted_->state_vector();
    Pc = Pp;

    if (param_->enable_correction)
    {
        Q = cv::Matx<double, 2, 2>(param_->measurement_noise_alpha, 0, 0, param_->measurement_noise_rho);
        for (size_t idx_measurement = 0; (idx_measurement < measurement_match_.size()); idx_measurement++)
        {
            int idx_map = measurement_match_[idx_measurement];
            cv::Matx<double, 2, 3> H;             /// Check slides
            cv::Matx<double, 2, 1> v;             /// Measurement error between prediction (known data) and detection --> Siegwart;
            cv::Matx<double, 2, 2> Si;            /// Check slides
            cv::Matx<double, 3, 2> K;             /// Kalman gain
            cv::Matx<double, 3, 1> dx;            /// State change
            cv::Matx<double, 1, 1> d_mahalanobis; // just for debugging reasons, not needed;

            /**
             * @ToDo pose correction
             * Pose correction must update the KalmanFilter::xc and KalmanFilter::Pc which represents the corrected pose with covariance
             * have a look into Siegwart 2011 section 5.6.8.5 Case study: Kalman filter localization with line feature extraction
             * Siegwart correction implementation
             */
            /* your code start */

            LineSegment2D line = measurement_linesegments_->data.at(idx_measurement);
            Polar2D zt = line.toPolar();
            double alpha;
            double rho;
            if(idx_map >= 0) {
                alpha = predicted_linesegments_.at(idx_map).toPolar().alpha();
                rho = predicted_linesegments_.at(idx_map).toPolar().rho();
            }
            if (line.length() > (xc[0] * cos(line.angle() + xc[1] * sin(line.angle()))))
            {
                if(idx_map < 0) {
                    alpha = line.angle() - x[2];
                    rho = line.length() - (x[0]*cos(line.angle()) + x[1]*sin(line.angle()));
                }

                H(0, 0) = 0;
                H(0, 1) = 0;
                H(0, 2) = -1;
                H(1, 0) = -cos(line.angle());
                H(1, 1) = -sin(line.angle());
                H(1, 2) = 0;
            }
            else
            {   
                if(idx_map < 0) {
                    alpha = line.angle() + M_PI - x[2];
                    rho = (x[0]*cos(line.angle()) + x[1]*sin(line.angle())) - line.length();
                }

                H(0, 0) = 0;
                H(0, 1) = 0;
                H(0, 2) = -1;
                H(1, 0) = cos(line.angle());
                H(1, 1) = sin(line.angle());
                H(1, 2) = 0;
            }
            Si = H * Pc * H.t() + Q;
            K = Pc * H.t() * Si.inv();
            v(0, 0) = angle_difference(zt.alpha(), alpha);
            v(0, 1) = zt.rho() - rho;
            xc += K * v;
            Pc = (Pc.eye() - K * H) * Pc;
            /* your code end   */
        }
    }
    if (!estimated_pose_)
        estimated_pose_ = make_shared<Pose2D>();
    estimated_pose_->set(xc);
    P = Pc;
}

void EKF::correction(StampedDataPtr<Points2D> z)
{
    measurement_laser_scan_ = z;
}

void EKF::data_association()
{
    measurement_match_.clear();
    if (!pose_predicted_)
        return;
    /**
     * @ToDo predicted linesegments
     * compute the measurement prediction (predicted_linesegments_)
     * The predicted_linesegments_ are map_linesegments_ in the robots pose prediction frame
     */
    /* your code start */
    predicted_linesegments_.clear();
    for(int i = 0; i < map_linesegments_.size(); i++) {
        Point2D p0 = pose_predicted_->tf().inv()*map_linesegments_.at(i).p0();
        Point2D p1 = pose_predicted_->tf().inv()*map_linesegments_.at(i).p1();
        LineSegment2D line_segment;
        line_segment.set(p0, p1);
        predicted_linesegments_.push_back(line_segment);
    }
    /* your code end   */

    if (!param_->enable_data_association)
        return;
    measurement_match_.resize(measurement_linesegments_->data.size(), -1);
    for (size_t i = 0; i < measurement_linesegments_->data.size(); i++)
    {
        Polar2D measurement = measurement_linesegments_->data[i].toPolar();
        float dMin = FLT_MAX;
        measurement_match_[i] = -1;
        double smallest_alpha = FLT_MAX;
        for (size_t j = 0; j < predicted_linesegments_.size(); j++)
        {
            Polar2D prediction = predicted_linesegments_[j].toPolar();
            /**
             * @ToDo matching measurement with prediction
             * find the best measurement prediction idx j and store it in measurement_match_[i]
             * Here the hough transform is used to match the lines as seen from the prediction (predicted_linesegments)
             * with the currently measured linesegments. Both are in the same coordinate space (prev step) and are transformed
             * into hough space (lines are points here and can be easily matched).
             * rqt allows for defining a matching threshold (distance)
             */
            /* your code start */
            // wie vergleicht man die?

            if(prediction.distanceTo(measurement) < dMin && 
                angle_difference(prediction.alpha(), measurement.alpha()) < param_->data_association_line_alpha &&
                abs(prediction.rho() - measurement.rho()) < param_->data_association_line_rho &&
                prediction.distanceTo(measurement) < param_->data_association_distance_to_endpoints)
            {
                dMin = prediction.distanceTo(measurement);
                measurement_match_[i] = j;
            }
            
            /*if (prediction.distanceTo(measurement) < dMin)
            {
                dMin = prediction.distanceTo(measurement);
                measurement_match_[i] = j;
            }*/
            /*(void) measurement;
            (void) dMin;
            (void) prediction;*/
            /* your code end   */
        }
    }
}