#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mr_pf/particle_filter.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

ParticleFilter::ParticleFilter()
    : generator_(rd_())
{
}
bool ParticleFilter::init(ParticleFilterParameterPtr param)
{
    param_ = param;

    map_header_.init(param_->map_cols, param_->map_rows,
                     param_->map_min_x, param_->map_max_x,
                     param_->map_min_y, param_->map_max_y,
                     param_->map_rotation + M_PI);

    if (map_header_.scale_x() != map_header_.scale_y())
    {
        std::cerr << "loadMap: non-symmetric scale!";
        return true;
    }
    cv::Mat image = cv::imread(param_->map_file, cv::IMREAD_GRAYSCALE);
    cv::resize(image, map_, cv::Size(param_->map_cols, param_->map_rows), cv::INTER_AREA);

    distance_field_pixel_.create(param_->map_cols, param_->map_rows);
    likelihood_field_.create(param_->map_cols, param_->map_rows);

    uniform_x_ = std::uniform_real_distribution<double>(param_->map_min_x, param_->map_max_x);
    uniform_y_ = std::uniform_real_distribution<double>(param_->map_min_y, param_->map_max_y);
    uniform_theta_ = std::uniform_real_distribution<double>(-M_PI, M_PI);
    uniform_ = std::uniform_real_distribution<double>(0., 1.);
    reset_ = Reset::UNIFORM;
    return false;
}

SamplePose2DPtr &ParticleFilter::sample_uniform(SamplePose2DPtr &des)
{
    des->x() = uniform_x_(generator_);
    des->y() = uniform_y_(generator_);
    des->theta() = uniform_theta_(generator_);
    des->recompute_cached_cos_sin();
    return des;
}
SamplePose2DPtr &ParticleFilter::sample_normal(SamplePose2DPtr &des, tuw::Pose2D &src, double sigma_position, double sigma_orientation)
{
    des->x() = src.x() + normal_distribution_(generator_) * sigma_position;
    des->y() = src.y() + normal_distribution_(generator_) * sigma_position;
    des->theta() = src.theta() + normal_distribution_(generator_) * sigma_orientation;
    des->recompute_cached_cos_sin();
    return des;
}
void ParticleFilter::set_ground_truth(const Pose2D &p)
{
    if (!ground_truth_)
        ground_truth_ = make_shared<Pose2D>(p);
    else
        *ground_truth_ = p;
}
void ParticleFilter::set_init_pose(const Pose2D &p)
{
    if (!init_pose_)
        init_pose_ = make_shared<Pose2D>(p);
    else
        *init_pose_ = p;
}

void ParticleFilter::reset_samples()
{

    switch (reset_)
    {
    case Reset::UNIFORM:
        for (auto &s : samples_)
        {
            sample_uniform(s);
        }
        break;
    case Reset::GROUND_TRUTH:
        if (ground_truth_)
        {
            for (auto &s : samples_)
            {
                sample_normal(s, *ground_truth_, param_->resample_noise_position, param_->resample_noise_orientation);
            }
        }
        break;
    case Reset::INTI_POSE:
        if (init_pose_)
        {
            for (auto &s : samples_)
            {
                sample_normal(s, *init_pose_, param_->resample_noise_position, param_->resample_noise_orientation);
            }
        }
        break;
    default:
        break;
    }
    if (!param_->continues_reset)
        reset_ = Reset::OFF;
}
void ParticleFilter::generate_samples()
{
    while (samples_.size() < param_->nr_of_samples)
    {
        SamplePose2DPtr s = std::make_shared<SamplePose2D>();
        samples_.push_back(sample_uniform(s));
    }
    while (samples_.size() > param_->nr_of_samples)
    {
        samples_.pop_back();
    }
}

void ParticleFilter::compute_weights(const vector<std::pair<tuw::Point2D, tuw::Polar2D>> &z_s, const tuw::Pose2D &pose_laser)
{
    std::chrono::steady_clock::time_point time_weighting_last_ = std::chrono::steady_clock::now();

    /// compute or update likelihood field
    compute_likelihood_field();

    /// reduce or increase number of samples
    generate_samples();

    tf_base_sensor_ = pose_laser.tf(); /// make a copy of the laser frame

    /// crate a copy of the current laser measurement
    z_r_.resize(z_s.size());
    vector<std::pair<tuw::Point2D, tuw::Polar2D>>::const_iterator src;
    vector<tuw::Point2D>::iterator des;
    for (src = z_s.begin(), des = z_r_.begin(); (src < z_s.end()) && (des < z_r_.end()); src++, des++)
        (*des) = (*src).first;

    std::vector<size_t> used_beams(std::floor(std::clamp(param_->rate_of_beams_used, 0., 1.) * (z_s.size() - 1))); /// vector of beam indexes used

    /**
     * @ToDo Sensor Model
     * Select beams either randomly or equally distributed with a if-else statement.
     **/
    if (param_->level >= 3)
    {
    }
    else
    {
        /**
         * @note your code here
         **/
        int step_size = z_s.size() / used_beams.size();
        for (size_t i = 0; i < used_beams.size(); i++)
        {
            size_t idx = i * step_size + step_size / 2.;
            used_beams[i] = idx;
        }
    }
    double weight_sum = 0;

    auto weight_sample = [&](SamplePtr<Pose2D> &s)
    {
        cv::Matx33d M = map_header_.Mw2m() * s->tf() * pose_laser.tf();
        double q = 1;
        for (size_t i : used_beams)
        {
            const std::pair<tuw::Point2D, tuw::Polar2D> &beam = z_s[i];
            /**
             * @ToDo Sensor Model
             * Computing the Weight
             * the used_beams should define the index of used laser beams
             * Since we are using c++11 and later you have to to use the function object / lambda expression weight_sample
             * --> http://en.cppreference.com/w/cpp/algorithm/for_each
             * --> http://en.cppreference.com/w/cpp/language/lambda
             * The function computes the likelihood field lookup for each laser measurement viewed
             * in the coordinate system of each Particle.
             * The likelihood values are then computed as in the book and lecture slides
             * The resulting weight is stored inside each particle.
             */
            if (param_->level > ParticleFilterParameter::COMPUTE_SAMPLES_WEIGHT)
            {
            }
            else
            {
                /**
                 * @note your code here
                 * Do not forget to check if the laser beam is inside the map
                 **/
                //(void)M;    /// to silence a warning about unused variables
                //(void)beam; /// to silence a warning about unused variables

                if(beam.second.distanceTo(s->position()) != param_->z_max &&
                    beam.first.get_x() >= param_->map_min_x && beam.first.get_x() <= param_->map_max_x &&
                    beam.first.get_y() >= param_->map_min_y && beam.first.get_y() <= param_->map_max_y) {
                    
                    /*double x_t = s->get_x() + beam.first.get_x()*cos(s->get_theta()) - 
                        beam.first.get_y()*sin(s->get_theta()) + 
                        beam.second.distanceTo(s->position())*cos(s->get_theta() + beam.first.angle());
                    double y_t = s->get_y() + beam.first.get_y()*cos(s->get_theta()) - 
                        beam.first.get_x()*sin(s->get_theta()) + 
                        beam.second.distanceTo(s->position())*sin(s->get_theta() + beam.first.angle());*/
                    double qr = param_->z_rand / param_->z_max;
                    // Point2D p = M * beam.first;

                    double qh = param_->z_hit * likelihood_field_.at<float>(s->get_x(), s->get_y()) + qr;

                    q *= qh;
                }
            }
        }
        s->weight() = q;
        weight_sum += s->weight();
    };
    std::for_each(samples_.begin(), samples_.end(), weight_sample);
    std::sort(samples_.begin(), samples_.end(), Sample<Pose2D>::greater);

    samples_weight_max_ = 0;
    for (size_t i = 0; i < samples_.size(); i++)
    {
        SamplePtr<Pose2D> &s = samples_[i];
        s->weight() /= weight_sum;
        s->idx() = i;
        if (s->weight() > samples_weight_max_)
            samples_weight_max_ = s->weight();
        // std::cout << s->idx() << ": " << s->weight() << std::endl;
    }
    processing_time_compute_weights_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_weighting_last_);
}

void ParticleFilter::compute_likelihood_field()
{
    static double sigma = 0.;
    static ParticleFilterParameter::Level level = ParticleFilterParameter::CHECK_MAP_FILE_EXISTS;
    /// Check to compute the likelihood field only if needed (changes on the parameter)
    if ((sigma == param_->sigma_hit) && (level == param_->level) && (param_->sigma_hit <= 0))
        return;
    sigma = param_->sigma_hit;
    level = param_->level;
    boost::math::normal normal_distribution(0, sigma);

    /**
     * @ToDo Sensor Model
     * using the cv::distanceTransform and the boost::math::pdf
     * Here, the likelihood field is computed using the following steps.
     * - First the distance transform is called on the map which outputs the distance to the nearest pixels holding the value 0
     *   of all other pixels. Use the matrix distance_field_pixel_ to store the result.
     * - To transform from pixels to metric values, the field has to be divided by scale_.
     *   Use the matrix distance_field_ to store the result.
     * - The pdf can then be used on the metric distance field to obtain the likelihood values.
     *   Use the matrix likelihood_field_ to store the result.
     **/
    if (param_->level > ParticleFilterParameter::COMPUTE_LIKELIHOOD_FIELD)
    {
    }
    else
    {
        /// @note your code
        /// replace the following lines with your code

        cv::distanceTransform(map_, distance_field_pixel_, cv::DIST_L2, cv::DIST_MASK_5, cv::DIST_LABEL_PIXEL);
        // Scale from px to meters ...
        distance_field_ = distance_field_pixel_/map_header_.scale_x();
        

        for (int r = 0; r < likelihood_field_.rows; r++)
        {
            for (int c = 0; c < likelihood_field_.cols; c++)
            {
                likelihood_field_(r, c) = boost::math::pdf(normal_distribution, distance_field_(r, c));
                // likelihood_field_(r, c) =

                // Dummy code below, replace with your code
                //float v = (float)c / (float)likelihood_field_.cols;
                //likelihood_field_(r, c) = v;
            }
        }
    }
}

void ParticleFilter::update(const Command2DConstPtr u, double dt)
{

    reset_samples();
    if (!u || param_->disable_update)
        return;
    std::chrono::steady_clock::time_point time_update_start = std::chrono::steady_clock::now();

    for (SamplePose2DPtr s : samples_)
    {
        /**
         * @ToDo Motion model
         * implement the forward sample_motion_velocity algorithm and be aware that w can be zero!!
         * use the param_->alpha1 - param_->alpha6 as noise parameters
         * Executes the forward prediction step for each particle
         **/
        if (param_->level > ParticleFilterParameter::MOTION_UPDATE)
        {
        }
        else
        {
            /// @node your code
            double v = u->v();
            double w = u->w();
            double x = s->get_x();
            double y = s->get_y();
            double theta = s->get_theta();
            double v_hut = v + normal_distribution_(generator_)*(param_->alpha1*v*v + param_->alpha2*w*w);
            double w_hut = w + normal_distribution_(generator_)*(param_->alpha3*v*v + param_->alpha4*w*w);
            double gamma_hut = normal_distribution_(generator_)*(param_->alpha5*v*v + param_->alpha6*w*w);
            double x_strich = x - v_hut/w_hut * sin(theta) + v_hut/w_hut * sin(theta + w_hut*dt);
            double y_strich = y + v_hut/w_hut * cos(theta) - v_hut/w_hut * cos(theta + w_hut*dt);
            double theta_strich = theta + w_hut*dt + gamma_hut*dt;
            ParticleFilter::estimated_pose_->set(x_strich, y_strich, theta_strich);
        }
    }
    dt += param_->duration_offset;
    processing_time_update_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_update_start);
}
void ParticleFilter::resample(double dt)
{
    if (samples_.empty())
        return;
    std::chrono::steady_clock::time_point time_resample_start = std::chrono::steady_clock::now();
    std::uniform_int_distribution<size_t> uniform_idx_des(0, samples_.size() - 1);

    std::vector<SamplePose2DPtr> samples_old = samples_;                            /// old sample set
    size_t M = samples_.size() * std::clamp<double>(param_->resample_rate, 0., 1.); /// number of cloned samples
    /**
     * @ToDo Resample
     * implement a resample wheel
     * Implementation of two different resampling steps is required for the exercise
     */
    if (param_->level > ParticleFilterParameter::RESAMPLING)
    {
    }
    else
    {
        /// @node your code
        (void)dt; /// to silence a warning about unused variables
        //(void)M;  /// to silence a warning about unused variables
        //param_->resample_strategy = ParticleFilterParameter::KEEP_BEST;
        if (param_->resample_strategy == ParticleFilterParameter::KEEP_BEST)
        {
            /// simple resample strategy witch keeps the best n-M best
            int smallest_idx[M] = {0};
            int smallest_cnt = 0;
            int largest_idx[M] = {0};
            int largest_cnt = 0;
            for(int i = 1; i < samples_old.size(); i++) {
                double weight = samples_old.at(i)->weight();
                double smallest = samples_old.at(smallest_idx[smallest_cnt])->weight();
                double largest = samples_old.at(largest_idx[largest_cnt])->weight();
                if(weight <= smallest) {
                    smallest_cnt++;
                    if(smallest_cnt == M) {
                        smallest_cnt = 0;
                    }
                    smallest_idx[smallest_cnt] = i;
                }
                if(weight <= largest) {
                    largest_cnt++;
                    if(largest_cnt == M) {
                        largest_cnt = 0;
                    }
                    largest_idx[largest_cnt] = i;
                }
            }
            while(samples_.size() > 0) {
                samples_.pop_back();
            }
            for(int i = 0; i < samples_old.size(); i++) {
                tuw::SamplePose2DPtr p = samples_old.at(i);
                samples_.push_back(p);

                for(smallest_cnt = 0; smallest_cnt < M; smallest_cnt++) {
                    if(smallest_idx[smallest_cnt] == i){
                        samples_.pop_back();
                    }
                }
                for(largest_cnt = 0; largest_cnt < M; largest_cnt++) {
                    if(largest_idx[largest_cnt] == i){
                        samples_.push_back(p);
                    }
                }
            }
        }
        else if (param_->resample_strategy == ParticleFilterParameter::LOW_VARIANCE)
        {
            /// low variance strategy
            while(samples_.size() > 0) {
                samples_.pop_back();
            }

            int r = rand()%((int) M);
            int c = samples_old.at(0).get()->weight();
            int i = 1;
            for(int m = 1; m < (int) M; m++) {
                int U = r + (int) (m-1)*1.0/((int) M);
                while (U > c && i < samples_old.size()-1) {
                    i = i + 1;
                    double weight = samples_old.at(i)->weight();
                    c = c + (int) weight;
                }

                if((int) samples_.capacity() > 0) {
                    tuw::SamplePose2DPtr p = samples_old.at(i);
                    //std::cout << "x: " << p.get()->get_x() << std::endl;
                    samples_.push_back(p);
                }
            }
        }
    }
    processing_time_resample_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_resample_start);
}
tuw::Pose2DPtr ParticleFilter::compute_estimated_pose()
{
    return estimated_pose_;
}
