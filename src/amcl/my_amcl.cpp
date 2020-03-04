/*!
 * * \brief the definition of front-ibeo-based localization class
 * * \author Shifan Zhu (zhushifan12@gmail.com)
 * * \date May 29, 2019
 */


#include "amcl/my_amcl.h"

namespace my
{
namespace localization
{
namespace ibeo
{

bool AmclSensor::UpdateAction(PfT *pf, AmclSensorData *data)
{
    (void) pf;
    (void)data;
    return false;
}

ObservationResult AmclSensor::UpdateSensor(PfT *pf, AmclSensorData *data, MapPoints& map_left_points_, MapPoints& map_right_points_)
{
    ObservationResult obs_res;
    obs_res.observation_dist_ = 0;
    obs_res.total_weight_ = 0;
    (void) pf;
    (void)data;
    (void)map_left_points_;
    (void)map_right_points_;
    return obs_res;
}

MyAmcl::MyAmcl()
    : alpha1_(), alpha2_(), alpha3_(), alpha4_(), alpha5_(),
z_hit_(), z_rand_(), sigma_hit_(),
laser_model_type_(),odom_model_type_()
{
    return;
}

MyAmcl::MyAmcl(uint16_t max_beam, MapT* map)
{
    time_ = 0.0;
    max_beams_ = max_beam;
    map_ = map;
}

MyAmcl::~MyAmcl()
{

}

Scalar MyAmcl::Normalize(Scalar z)
{
  return atan2(sin(z),cos(z));
}

Scalar MyAmcl::AngleDiff(Scalar a, Scalar b)
{
    Scalar d1, d2;
    a = Normalize(a);
    b = Normalize(b);
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
        d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
        return(d1);
    else
        return(d2);
}

void MyAmcl::SetOdomModelDiff(Scalar alpha1, 
                       Scalar alpha2, 
                       Scalar alpha3, 
                       Scalar alpha4)
{
    this->odom_model_type_ = ODOM_MODEL_DIFF;
    this->alpha1_ = alpha1;
    this->alpha2_ = alpha2;
    this->alpha3_ = alpha3;
    this->alpha4_ = alpha4;
}

void MyAmcl::SetOdomModelOmni(Scalar alpha1,
                                Scalar alpha2,
                                Scalar alpha3, 
                                Scalar alpha4,
                                Scalar alpha5)
{
    this->odom_model_type_ = ODOM_MODEL_OMNI;
    this->alpha1_ = alpha1;
    this->alpha2_ = alpha2;
    this->alpha3_ = alpha3;
    this->alpha4_ = alpha4;
    this->alpha5_ = alpha5;
}

void MyAmcl::SetOdomModel(OdomModelT type,
                            Scalar alpha1,
                            Scalar alpha2,
                            Scalar alpha3,
                            Scalar alpha4,
                            Scalar alpha5)
{
    this->odom_model_type_ = type;
    this->alpha1_ = alpha1;
    this->alpha2_ = alpha2;
    this->alpha3_ = alpha3;
    this->alpha4_ = alpha4;
    this->alpha5_= alpha5;
}

void MyAmcl::SetModelLikelihoodField(Scalar z_hit,
                                       Scalar z_rand,
                                       Scalar sigma_hit)
{
    this->laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
    this->z_hit_ = z_hit;
    this->z_rand_ = z_rand;
    this->sigma_hit_ = sigma_hit;
//   map_update_cspace(this->map, max_occ_dist); //this function is used when using grid map
}

void MyAmcl::SetModelDistance(Scalar z_hit,
                                Scalar z_rand,
                                Scalar sigma_hit)
{
    this->laser_model_type_ = LASER_MODEL_DISTANCE;
    this->z_hit_ = z_hit;
    this->z_rand_ = z_rand;
    this->sigma_hit_ = sigma_hit;
//   map_update_cspace(this->map, max_occ_dist); //this function is used when using grid map
}



bool MyAmcl::UpdateAction(PfT *pf, AmclSensorData *data)
{
    AmclOdomData *odata;
    odata = (AmclOdomData*) data;
    PfSampleSetT *set;
    set = pf->sets + pf->current_set;
    PfVectorT old_pose = PfVectorSub(odata->pose_,odata->delta_);
    switch(this->odom_model_type_)
    {
    case ODOM_MODEL_OMNI:
    {
        ;
    }
    break;
    case ODOM_MODEL_DIFF:
    {
        // Implement sample_motion_odometry (Prob Rob p 136)
        Scalar delta_rot1, delta_trans, delta_rot2;
        Scalar delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
        Scalar delta_rot1_noise, delta_rot2_noise;
        Scalar r;//Lateral errors

        // Avoid computing a bearing from two poses that are extremely near each
        // other (happens on in-place rotation).
        if(sqrt(odata->delta_.v[1]*odata->delta_.v[1] + 
                odata->delta_.v[0]*odata->delta_.v[0]) < 0.01)
            delta_rot1 = 0.0;
        else
            delta_rot1 = AngleDiff(atan2(odata->delta_.v[1], odata->delta_.v[0]),
                                old_pose.v[2]);
        delta_trans = sqrt(odata->delta_.v[0]*odata->delta_.v[0] +
                        odata->delta_.v[1]*odata->delta_.v[1]);
        delta_rot2 = AngleDiff(odata->delta_.v[2], delta_rot1);

        // We want to treat backward and forward motion symmetrically for the
        // noise model to be applied below.  The standard model seems to assume
        // forward motion.
        delta_rot1_noise = std::min(fabs(AngleDiff(delta_rot1,0.0)),
                                    fabs(AngleDiff(delta_rot1,M_PI)));
        delta_rot2_noise = std::min(fabs(AngleDiff(delta_rot2,0.0)),
                                    fabs(AngleDiff(delta_rot2,M_PI)));

        for (uint16_t i = 0; i < set->sample_count; i++)
        {
            PfSampleT* sample = set->samples + i;

            // Sample pose differences
            delta_rot1_hat = AngleDiff(delta_rot1,
                                        PfRanGaussian(this->alpha1_*delta_rot1_noise*delta_rot1_noise +
                                                        this->alpha2_*delta_trans*delta_trans));
            delta_trans_hat = delta_trans - 
                    PfRanGaussian(this->alpha3_*delta_trans*delta_trans +
                                    this->alpha4_*delta_rot1_noise*delta_rot1_noise +
                                    this->alpha4_*delta_rot2_noise*delta_rot2_noise);
            delta_rot2_hat = AngleDiff(delta_rot2,
                                        PfRanGaussian(this->alpha1_*delta_rot2_noise*delta_rot2_noise +
                                                        this->alpha2_*delta_trans*delta_trans));

            r = drand48() - 0.5;
            // Apply sampled update to particle pose
            sample->pose.v[0] += delta_trans_hat * 
                    cos(sample->pose.v[2] + delta_rot1_hat) + r * sin(sample->pose.v[2]);
            sample->pose.v[1] += delta_trans_hat * 
                    sin(sample->pose.v[2] + delta_rot1_hat) - r * cos(sample->pose.v[2]);
            sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
        }
    }
    break;
    }
    return true;
}

ObservationResult MyAmcl::UpdateSensor(PfT *pf, AmclSensorData *data, MapPoints& map_left_points_, MapPoints& map_right_points_)
{
    ObservationResult obs_res;
    if(this->laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD)
        obs_res = PfUpdateSensor(pf, (PfSensorModelFnT)LikelihoodFieldModel, data, map_left_points_, map_right_points_);
    else if(this->laser_model_type_ == LASER_MODEL_DISTANCE)
        obs_res = PfUpdateSensor(pf, (PfSensorModelFnT)DistanceModel, data, map_left_points_, map_right_points_);
    else
        return obs_res;
    return obs_res;
}

ObservationResult MyAmcl::DistanceModel(AmclLaserData *data, PfSampleSetT *set, MapPoints& map_left_points_, MapPoints& map_right_points_)
{
    (void) map_left_points_;//Don't use left points now
    MyAmcl *self;
    uint16_t i, j, step;
    Scalar avarage_distance;//avarage_distance for every particle
    Scalar min_distance;//min avarage_distance for all particle
    Scalar z, pz;
    Scalar p;
    Scalar obs_range, obs_bearing;
    ObservationResult obs_res;
    PfSampleT *sample;
    PfVectorT pose;
    PfVectorT hit;

    self = (MyAmcl*) data->sensor;

    obs_res.total_weight_ = 0.0;
    static my::Quaternion quat_ibeo_to_novatel(0.6931309994093054,0.009103617538894484,-0.00377234507077584,0.7207442758830832);//92.265
    static my::Quaternion quat_novatel_to_body(0.7071067811865476,0,0,-0.7071067811865475);//-90
    
    static Scalar yaw_ibeo_to_novatel = std::atan2(2*(quat_ibeo_to_novatel.w()*quat_ibeo_to_novatel.z() + quat_ibeo_to_novatel.x()*quat_ibeo_to_novatel.y()), 
    (1 - 2*(quat_ibeo_to_novatel.y()*quat_ibeo_to_novatel.y() + quat_ibeo_to_novatel.z()*quat_ibeo_to_novatel.z())));
    static Scalar yaw_novatel_to_body = std::atan2(2*(quat_novatel_to_body.w()*quat_novatel_to_body.z() + quat_novatel_to_body.x()*quat_novatel_to_body.y()), 
    (1 - 2*(quat_novatel_to_body.y()*quat_novatel_to_body.y() + quat_novatel_to_body.z()*quat_novatel_to_body.z())));

    static my::Rot3 rot3_ibeo_to_novatel = quat_ibeo_to_novatel.toRotationMatrix();
    static my::Rot3 rot3_novatel_to_body = quat_novatel_to_body.toRotationMatrix();
    static my::Pose2 T_ni = my::Pose2(yaw_ibeo_to_novatel,-0.003632513600639509, 3.695114608837488);//-0.003632513600639509, 3.695114608837488
    static my::Pose2 T_bn = my::Pose2(yaw_novatel_to_body,0,0);

    // std::vector<Point2Type> point2s_left;
    std::vector<Point2Type> point2s_right;
    // Point2Type point2_left;
    Point2Type point2_right;

    // for(uint16_t map_left_points_cnt = 0; map_left_points_cnt < map_left_points_.size(); map_left_points_cnt++)
    // {
    //     point2_left.Set(map_left_points_[map_left_points_cnt].x(),map_left_points_[map_left_points_cnt].y());
    //     point2s_left.push_back(point2_left);
    // }
    for(uint16_t map_right_points_cnt = 0; map_right_points_cnt < map_right_points_.size(); map_right_points_cnt++)
    {
        point2_right.Set(map_right_points_[map_right_points_cnt].x(),map_right_points_[map_right_points_cnt].y());
        point2s_right.push_back(point2_right);
    }

    min_distance = 1000;
    // Compute the sample weights
    for (j = 0; j < set->sample_count; j++)
    {
        sample = set->samples + j;
        pose = sample->pose;//prior pose which was updated by motion model

        Point2Type point_tran(pose.v[0],pose.v[1]);
        my::Pose2 T_wx(pose.v[2],point_tran);

        // std::vector<Point2Type> point2s_fit_left;
        std::vector<Point2Type> point2s_fit_right;

        // for(uint16_t point2_cnt = 0; point2_cnt < point2s_left.size(); point2_cnt+=2)
        // {
        //     Point2Type point2(T_wx.Inverse()*point2s_left[point2_cnt]);
        //     point2s_fit_left.push_back(point2);
        // }
        for(uint16_t point2_cnt = 0; point2_cnt < point2s_right.size(); point2_cnt+=2)
        {
            Point2Type point2(T_wx.Inverse()*point2s_right[point2_cnt]);
            point2s_fit_right.push_back(point2);
        }

        p = 1.0;

        // PolyFit2Type poly_fit_left(2);
        // bool fit_status_left = poly_fit_left.Fit(point2s_fit_left);
        // LOG(INFO) << "Residual_local_right = "<<poly_fit_left.Residual();

        PolyFit2Type poly_fit_right(2);
        bool fit_status_right = poly_fit_right.Fit(point2s_fit_right);
        if(j == 0)
        {
            LOG(INFO) << "Residual_local_right = "<<poly_fit_right.Residual();
        }

        // if(!fit_status_left && !fit_status_right)
            // LOG(WARNING) << "Can not fit both side road points";
        if(!fit_status_right)
            LOG(WARNING) << "Can not fit right road points";

        // Pre-compute a couple of things
        Scalar z_hit_denom = 2 * 2 * 2;//self->sigma_hit_ * self->sigma_hit_;TODO sigma_hit isn't correct value
        Scalar z_rand_mult = 1.0/data->range_max_;

        // this is related to sample rate of lidar points
        step = (data->range_count_ - 1) / (self->max_beams_ - 1);

        // Step size must be at least 1
        if(step < 1)
            step = 1;

        avarage_distance = 0;
        for (i = 0; i < data->range_count_; i += step)
        {
            obs_range = data->ranges[i][0];
            obs_bearing = data->ranges[i][1];

            // This model ignores max range readings
            if(obs_range >= data->range_max_)
                continue;

            // Check for NaN
            if(obs_range != obs_range)
                continue;

            pz = 0.0;

            // Compute the endpoint of the beam
            hit.v[0] = obs_range * cos(obs_bearing);
            hit.v[1] = obs_range * sin(obs_bearing);

            // if(i % 100 == 0){
            //     std::cout<<"obs_bearing = "<<obs_bearing<<std::endl;
            // }
            Scalar dist_to_curve;
            Point2Type lidar_point(hit.v[0],hit.v[1]*0.85);


            if(obs_bearing > 0)
            {
                // poly_fit_left.ComputePointDistanceToCurve(lidar_point, dist_to_curve);
            }
            else
            {
                poly_fit_right.ComputePointDistanceToCurve(lidar_point, dist_to_curve);
            }

            z = dist_to_curve;
            obs_res.observation_dist_ = z;
            avarage_distance += z;
        
            // Gaussian model
            // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
            pz += 0.5 * exp(-(z * z) / z_hit_denom);
            // Part 2: random measurements
            pz += 0.5 * z_rand_mult;

            assert(pz <= 1.0);
            assert(pz >= 0.0);
            // p *= pz;
            // here we have an ad-hoc weighting scheme for combining beam probs
            // works well, though...
            p += pz*pz*pz;
        }
        avarage_distance = avarage_distance/data->range_count_*step;
        if(min_distance > avarage_distance)
        {
            min_distance = avarage_distance;
        }

        sample->weight *= p;
        obs_res.total_weight_ += sample->weight;
    }
    obs_res.observation_dist_ = min_distance;

    return(obs_res);
}

Scalar MyAmcl::LikelihoodFieldModel()
{
    Scalar nosense = 0.0;
    return nosense;
}

PfVectorT MyAmcl::UniformPoseGenerator(void* arg)
{
    (void) arg;
    //we don't use grid map.
    // MapT* map = (MapT*) arg;
    // free_space_indices.resize(10000);
    // for(int i = 0; i < map_->size_x; i++)
    //     for(int j = 0; j < map_->size_y; j++)
    //         if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
    //             free_space_indices.push_back(std::make_pair(i,j));
    // uint16_t rand_index = drand48() * free_space_indices.size();
    // std::pair<int16_t,int16_t> free_point = free_space_indices[rand_index];
    PfVectorT p;
    p.v[0] = drand48() * 10000;
    p.v[1] = drand48() * 10000;
    p.v[2] = drand48() * 2 * M_PI -M_PI;
    return p;
}

} // ibeo namespace
} // localization namespace
} // my namespace
