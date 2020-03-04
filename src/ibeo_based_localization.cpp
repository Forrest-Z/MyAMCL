/*!
 * * \brief the definition of front-ibeo-based localization class
 * * \author Shifan Zhu (zhushifan12@gmail.com)
 * * \date May 29, 2019
 */


#include <ibeo_based_localization.h>

namespace my
{
namespace localization
{
namespace ibeo
{

////////////////////////////////////////////////////////////////////////////////
IbeoBasedLocalization::
IbeoBasedLocalization(const Parameters& parameters,  const Options& options)
    : solution_(), parameters_(parameters), options_(options), cur_time_(0,0), last_time_(0,0),
      ground_truth_odometry_(), unoptimized_odometry_(),
      latest_ibeo_scan_data_(), fine_odometry_(),
      map_left_points_(), map_right_points_(), pf_init_(false),
      pf_(nullptr), laser_(nullptr), last_unoptimized_pose_(), last_optimized_pose_()
{
    input_manager_.reset(new MeasurementsManagerIbeo);
    my_amcl_ptr_.reset(new my);

    localization_thread_ =
        std::shared_ptr<std::thread>(
            new std::thread(std::bind(&IbeoBasedLocalization::ThreadFunc,
                                      this, std::placeholders::_1),
                            parameters_.estimation_frequence_));

    state_ = State::READY;
}

////////////////////////////////////////////////////////////////////////////////
IbeoBasedLocalization::~IbeoBasedLocalization()
{
    if (localization_thread_ != nullptr)
    {
        localization_thread_->join();
    }
}

////////////////////////////////////////////////////////////////////////////////
bool IbeoBasedLocalization::CheckPreCondition()
{
    input_manager_->GetCurrentGroundTruth(cur_time_, ground_truth_odometry_);
    return  GotLatestIbeoScanData() && GotLatestRoadSideMapPoints() && GotLatestPriorOdometry();
}


////////////////////////////////////////////////////////////////////////////////
void IbeoBasedLocalization::EstimationCore()
{
    PfVectorT pose;

    bool resample = false;

    pose.v[0] = unoptimized_odometry_.pose().x();
    pose.v[1] = unoptimized_odometry_.pose().y();
    pose.v[2] = atan2(unoptimized_odometry_.pose().Rotation().R10(),
                      unoptimized_odometry_.pose().Rotation().R00());

    PfVectorT delta = PfVectorZero();

    // if was not initialized
    if(!pf_init_)
    {
        if (!SetOdomAndSensorModel())
        {
            return;
        }

        PfVectorT init_pose_mean;
        init_pose_mean.v[0] = unoptimized_odometry_.pose().x();
        init_pose_mean.v[1] = unoptimized_odometry_.pose().y();
        init_pose_mean.v[2] = atan2(unoptimized_odometry_.pose().Rotation().R10(),
                                    unoptimized_odometry_.pose().Rotation().R00());

        pf_ = PfAlloc(parameters_.min_particles_, parameters_.max_particles_,
                parameters_.alpha_slow_, parameters_.alpha_fast_,
                (PfInitModelFnT)(my_amcl_ptr_->UniformPoseGenerator),
                (void *)map_);
        pf_->pop_err = parameters_.pf_err_;
        pf_->pop_z = parameters_.pf_z_;

        PfMatrixT init_pose_cov = PfMatrixZero();
        init_pose_cov.m[0][0] = parameters_.initial_cov_aa_;
        init_pose_cov.m[1][1] = parameters_.initial_cov_xx_;
        init_pose_cov.m[2][2] = parameters_.initial_cov_yy_;

        PfInit(pf_, init_pose_mean, init_pose_cov);
        last_unoptimized_pose_ = pose;
        last_optimized_pose_ = pose;
        pf_init_ = true;
    }

    // if was initialized
    if(pf_init_)
    {
        delta.v[0] = pose.v[0] - last_unoptimized_pose_.v[0]; //difference of x-axis between current pose and last pose
        delta.v[1] = pose.v[1] - last_unoptimized_pose_.v[1];
        delta.v[2] = my_amcl_ptr_->AngleDiff(pose.v[2],last_unoptimized_pose_.v[2]);//difference of angles between current pose and last pose

        parameters_.update_pose_ = fabs(delta.v[0]) > parameters_.delta_thresh_ ||
                                   fabs(delta.v[1]) > parameters_.delta_thresh_ ||
                                   fabs(delta.v[2]) > parameters_.delta_thresh_;

        Scalar r00 = unoptimized_odometry_.pose().Rotation().R00();
        Scalar r10 = unoptimized_odometry_.pose().Rotation().R10();
        Scalar r20 = unoptimized_odometry_.pose().Rotation().R20();
        Scalar r21 = unoptimized_odometry_.pose().Rotation().R21();
        Scalar r22 = unoptimized_odometry_.pose().Rotation().R22();
        yawpitchroll_.v[0] = atan2(r10,r00);
        yawpitchroll_.v[1] = atan2(-r20,sqrt(r21*r21+r22*r22));
        yawpitchroll_.v[2] = atan2(r21,r22);
    }

    if(pf_init_ && parameters_.update_pose_)
    {
        /// set odom data
        AmclOdomData odata;
        odata.pose_ = last_optimized_pose_;
        odata.delta_ = delta;
        my_amcl_ptr_->UpdateAction(pf_,(AmclSensorData*)&odata);

        // set laser data
        AmclLaserData ldata;
        delete laser_;
        laser_ = new my(parameters_.max_beams_, map_);
        ldata.sensor = new my(*laser_);
        ldata.range_max_ = 10000;
        uint16_t ldata_cnt = 0;
        uint16_t angle_ticks = latest_ibeo_scan_data_.scan_data_info.angle_ticks;
        for(uint16_t i = 0; i<latest_ibeo_scan_data_.scan_point_list.size(); i++)
        {
            uint8_t layer = latest_ibeo_scan_data_.scan_point_list[i].layer_echo & 0x07;
            float32_t distance = latest_ibeo_scan_data_.scan_point_list[i].distance / 100.0;
            int16_t horizontal_angle = latest_ibeo_scan_data_.scan_point_list[i].horizontal_angle;
            float64_t theta = (float64_t)horizontal_angle / angle_ticks * 2 * M_PI;
            if((layer == 2) && (distance < 50 && abs(distance * std::sin(theta))<10))
            {
                ldata_cnt++;
            }
        }
        ldata.range_count_ = ldata_cnt;
        ldata.ranges = new Scalar[ldata_cnt][2];
        uint16_t ldata_range_cnt = 0;
        for(uint16_t i = 0; i<latest_ibeo_scan_data_.scan_point_list.size(); i++)
        {
            uint8_t layer = latest_ibeo_scan_data_.scan_point_list[i].layer_echo & 0x07;
            float32_t distance = latest_ibeo_scan_data_.scan_point_list[i].distance / 100.0;
            int16_t horizontal_angle = latest_ibeo_scan_data_.scan_point_list[i].horizontal_angle;
            float64_t theta = (float64_t)horizontal_angle / angle_ticks * 2 * M_PI;
            if((layer == 2) && (distance < 50 && abs(distance * std::sin(theta))<10))
            {
                ldata.ranges[ldata_range_cnt][0] = distance;
                ldata.ranges[ldata_range_cnt][1] = theta;
                ldata_range_cnt++;
            }
        }

        ObservationResult obs_res;
        obs_res = my_amcl_ptr_->UpdateSensor(pf_, (AmclSensorData*)&ldata, map_left_points_, map_right_points_);
        if(obs_res.observation_dist_>2)
        {
            pf_init_ = false;
        }

        /// Resample the particles, based on particle's weight
        PfUpdateResample(pf_);
        parameters_.update_pose_ = false;
        resample = true;
        last_unoptimized_pose_ = pose;
    }

    if(resample)
    {
        resample = false;
        // Read out the current hypotheses
        Scalar max_weight = 0.0;
        int max_weight_hyp = -1;
        std::vector<AmclHypT> hyps;
        hyps.resize(pf_->sets[pf_->current_set].cluster_count);
        for(int hyp_count = 0; hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
        {
            Scalar weight;
            PfVectorT pose_mean;
            PfMatrixT pose_cov;
            if (!PfGetClusterStats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
            {
                LOG(ERROR) << "Couldn't get stats on cluster.";
                break;
            }

            hyps[hyp_count].weight = weight;
            hyps[hyp_count].pf_pose_mean = pose_mean;
            hyps[hyp_count].pf_pose_cov = pose_cov;

            last_optimized_pose_ = pose_mean;
            // if(abs(pose_mean.v[0]-unoptimized_odometry_.pose().x())>5 ||
            //    abs(pose_mean.v[1]-unoptimized_odometry_.pose().y())>2 ||
            //    abs(pose_mean.v[2]-atan2(unoptimized_odometry_.pose().Rotation().R10(),
            //    unoptimized_odometry_.pose().Rotation().R00()))>0)
            // {
            //     pf_init_ = false;
            // }

            if(hyps[hyp_count].weight > max_weight)
            {
                max_weight = hyps[hyp_count].weight;
                max_weight_hyp = hyp_count;
            }
        }

        if(max_weight > 0.0)
        {
            LOG(INFO) << "Max weight pose: "
                      << hyps[max_weight_hyp].pf_pose_mean.v[0]<<" "
                      << hyps[max_weight_hyp].pf_pose_mean.v[1]<<" "
                      << hyps[max_weight_hyp].pf_pose_mean.v[2];

            uint16_t mean_z = 0;
            for(uint16_t i = 0; i < map_left_points_.size(); i++)
            {
                mean_z += map_left_points_[i].z();
            }
            mean_z = mean_z / map_left_points_.size() + 2.8;//2.8 is the height from lans to body.Inaccurate.

            Pose3 cluster_post_mean(my::Pose3(my::Rot3::RzRyRx(yawpitchroll_.v[2],yawpitchroll_.v[1],hyps[max_weight_hyp].pf_pose_mean.v[2]),
                my::Point3(hyps[max_weight_hyp].pf_pose_mean.v[0],hyps[max_weight_hyp].pf_pose_mean.v[1],mean_z)));
            solution_.optimized_pose = cluster_post_mean;
            solution_.covariance = Matrix6::Zero();//TODO
            // solution_.position_covariance = Matrix6::Zero();
            solution_.optimized = true;
        }
        else
        {
            LOG(ERROR) << "No pose!";
        }
    }
    return ;
}

////////////////////////////////////////////////////////////////////////////////
bool IbeoBasedLocalization::PostProcess()
{
    ////////////////////////////////////////////////////////////////////////////
    // publish odometry.
    if (solution_.optimized == true)
    {
        PublishOptimizedOdometry();
    }
    else
    {
        LOG(ERROR) << "Optimization failed.";
    }

    ////////////////////////////////////////////////////////////////////////////

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool IbeoBasedLocalization::GotLatestPriorOdometry()
{
    bool got_prior_odometry =
        input_manager_->GetCurrentPriorOdometry(cur_time_, unoptimized_odometry_);

    if (got_prior_odometry == false)
    {
        LOG(WARNING) << "Cannot get the latest prior odometry for optimization.";
        return false;
    }
    
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool IbeoBasedLocalization::GotLatestRoadSideMapPoints()
{
    RouteInfo route_info;
    bool got_detected_lanes = input_manager_->GetLatestMeasurement(route_info);
    if(!got_detected_lanes)
    {
        return false;
    }

    LinkInfos link_infos = route_info.GetLinks();
    if(link_infos.size() == 0)
    {
        LOG(WARNING) << "Cannot get link info.";
        return false;
    }

    LinkInfo link_info = link_infos[0];
    Scalar link_length = link_info.GetLength();
    if (link_length <= parameters_.min_lane_length_)
    {
        LOG(WARNING) << "Link length is too short.";
        return false;
    }

     // Get and sort lane infos.
    LaneInfos lane_infos = link_info.GetLaneInfos();

    if(lane_infos.size() == 0)
    {
        return false;
    }
    
    std::sort(lane_infos.begin(), lane_infos.end(),
              [](const LaneInfo & a, const LaneInfo & b)
    {
        return a.GetSeq() < b.GetSeq();
    });

    LaneMarkInfo left_mark = lane_infos[0].GetLeftMark();
    LaneMarkInfo right_mark = lane_infos[lane_infos.size()-1].GetRightMark();

    map_left_points_ = left_mark.GetRefPoints();
    map_right_points_ = right_mark.GetRefPoints();

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool IbeoBasedLocalization::GotLatestIbeoScanData()
{
    bool got_ibeo_scan_data = 
        input_manager_->GetLatestIbeoScanData(latest_ibeo_scan_data_);

    // Get timestamp of detected lanes as current time of system
    cur_time_ = latest_ibeo_scan_data_.header.stamp();

    if(got_ibeo_scan_data == false)
    {
        LOG(WARNING) << "Cannot get latest ibeo scan data.";
        return false;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
const IbeoBasedLocalization::Parameters&
IbeoBasedLocalization::Params() const
{
    return parameters_;
}

////////////////////////////////////////////////////////////////////////////////
IbeoBasedLocalization::Parameters::
Parameters()
       : estimation_frequence_(1.0),
         max_beams_(30),
         min_particles_(500),
         max_particles_(600),
         alpha1_(0.1),
         alpha2_(0.02),
         alpha3_(0.08),
         alpha4_(0.08),
         alpha5_(0.0),
         z_hit_(0.5),
         z_rand_(0.5),
         sigma_hit_(2.0),
         lambda_short_(0.1),
         alpha_slow_(0.001),
         alpha_fast_(0.1),
         pf_err_(0.01),
         pf_z_(3),
         delta_thresh_(0.2),
        //  x_err_thresh_(5.0),
        //  y_err_thresh_(2.0),
        //  yaw_err_thresh_(0.0872664),//5°
         min_lane_length_(1.0),
         update_pose_(false),
         laser_model_type_string_("LASER_MODEL_DISTANCE"),
         odom_model_type_string_("ODOM_MODEL_DIFF"),
         initial_cov_xx_(0.25),
         initial_cov_yy_(0.25),
         initial_cov_aa_(0.00001)
{

}

////////////////////////////////////////////////////////////////////////////////
IbeoBasedLocalization::Options::
Options(const bool& _will_record_evaluation_files)
    : will_record_evaluation_files(_will_record_evaluation_files)
{

}

////////////////////////////////////////////////////////////////////////////////
bool IbeoBasedLocalization::SetOdomAndSensorModel()
{
    if(parameters_.laser_model_type_string_ == "LASER_MODEL_DISTANCE")
    {
        parameters_.laser_model_type_ = LASER_MODEL_DISTANCE;
        my_amcl_ptr_->SetModelDistance(parameters_.z_hit_, parameters_.z_rand_,parameters_.sigma_hit_);
    }
    else if(parameters_.laser_model_type_string_ == "LASER_MODEL_LIKELIHOOD_FIELD")
    {
        parameters_.laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
        my_amcl_ptr_->SetModelLikelihoodField(parameters_.z_hit_, parameters_.z_rand_,parameters_.sigma_hit_);
    }
    else
    {
        return false;
    }

    if(parameters_.odom_model_type_string_ == "ODOM_MODEL_DIFF")
    {
        parameters_.odom_model_type_ = ODOM_MODEL_DIFF;
        my_amcl_ptr_->SetOdomModel(parameters_.odom_model_type_, parameters_.alpha1_, 
            parameters_.alpha2_, parameters_.alpha3_, parameters_.alpha4_, parameters_.alpha5_);
    }
    else if(parameters_.odom_model_type_string_ == "ODOM_MODEL_OMNI")
    {
        parameters_.odom_model_type_ = ODOM_MODEL_OMNI;
        my_amcl_ptr_->SetOdomModel(parameters_.odom_model_type_, parameters_.alpha1_, 
            parameters_.alpha2_, parameters_.alpha3_, parameters_.alpha4_, parameters_.alpha5_);
    }
    else
    {
        return false;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
int32_t IbeoBasedLocalization::ProcessRouteInfo(const RouteInfo& route_info)
{
    input_manager_->AddMeasurement(route_info);
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
int32_t IbeoBasedLocalization::ProcessPriorOdometry(const Odometry& odometry)
{
    input_manager_->AddPriorOdometry(odometry);
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
int32_t IbeoBasedLocalization::ProcessIbeoScanData(const IbeoScanData& received_ibeo_scan_data)
{
    input_manager_->AddIbeoScanData(received_ibeo_scan_data);
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
int32_t IbeoBasedLocalization::ProcessGroundTruth(const Odometry& odometry)
{
    input_manager_->AddGroundTruth(odometry);
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
void IbeoBasedLocalization::
SetIbeoOdometryCallback(const OdometryCallback& odometry_callback)
{
    ibeo_odometry_callback_ = odometry_callback;
}

////////////////////////////////////////////////////////////////////////////////
void IbeoBasedLocalization::PublishOptimizedOdometry()
{
    static uint32_t count = 0;

    Pose3 optimized_pose = solution_.optimized_pose;

    Odometry optimized_odometry = Odometry();
    optimized_odometry.pose(optimized_pose);
    optimized_odometry.pose_covariance(Matrix6::Zero());

    optimized_odometry.header().seq(count++);
    optimized_odometry.header().stamp(cur_time_);
    optimized_odometry.header().coord_id(Coord::Value::WORLD);
    optimized_odometry.child_coord_id(Coord::Value::BODY);
    optimized_odometry.source(Odometry::Source::LIDAR);
    optimized_odometry.status(0);

    ibeo_odometry_callback_(optimized_odometry);
}

////////////////////////////////////////////////////////////////////////////////
IbeoBasedLocalization::SolutionState::SolutionState()
    : optimized_pose(Rot3::Identity(),Point3::Zero()), covariance(Matrix6::Identity()),
      optimized(false)
{

}

} // ibeo namespace
} // localization namespace
} // my namespace
