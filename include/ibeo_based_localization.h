/*!
 * * \brief the definition of front-ibeo-based localization class
 * * \author Shifan Zhu (zhushifan12@gmail.com)
 * * \date May 29, 2019
 */

#ifndef IBEO_BASED_LOCALIZATION_H_
#define IBEO_BASED_LOCALIZATION_H_


#include <thread>
#include <common/lane_boundaries.h>
#include <common/odometry.h>
#include <geometry/polyfit2.h>
#include <geometry/pose2.h>
#include <geometry/pose3.h>
#include <geometry/rot2.h>
#include <geometry/rot3.h>
#include <localization/common/localization_base.h>
#include <amcl/my_amcl.h>
#include <measurements_manager_ibeo.h>
#include <map/global_lane_boundary_list.h>
#include <numerics/eigen.h>
#include <utils/yaml.h>

namespace my
{
namespace localization
{
namespace ibeo
{

class MyAmcl;
class IbeoBasedLocalization : public LocalizationBase
{
private:

    /// @brief Define type name
    typedef LocalizationBase Base;
    using Base::Scalar;
    using Base::Rot3;
    using Base::Point3;
    using Base::Pose3;
    using Base::Vector3;
    using Base::Matrix3;

    typedef my::Rot2T<Scalar> Rot2;
    typedef my::Point2T<Scalar> Point2;
    typedef my::Pose2T<Scalar> Pose2;
    typedef my::PolyFit2T<Scalar> PolyFit;

    typedef my::Vector2T<Scalar> Vector2;
    typedef my::Vector4T<Scalar> Vector4;
    typedef my::VectorT<Scalar> VectorN;
    typedef my::Matrix2T<Scalar> Matrix2;
    typedef my::Matrix6T<Scalar> Matrix6;
    typedef my::Matrix4NT<Scalar> Matrix4N;

    typedef my::Odometry Odometry;

    typedef my::map::CurrentRouteInfo RouteInfo;
    typedef my::map::LinkInfo LinkInfo;
    typedef std::vector<LinkInfo> LinkInfos;
    typedef my::map::LaneInfo LaneInfo;
    typedef std::vector<LaneInfo> LaneInfos;
    typedef my::map::LaneMarkInfo LaneMarkInfo;
    typedef std::vector<Point3> MapPoints;
    typedef my::sensors::ibeo::IbeoScanData2202 IbeoScanData;

    typedef my::map::GlobalLaneBoundary MapLane;
    typedef my::map::GlobalLaneBoundaryList MapLanes;

    typedef MeasurementsManagerIbeo::Ptr MeasurementsManagerIbeoPtr;
    typedef MyAmcl::Ptr MyAmclPtr;

    typedef std::function<int32_t(const Odometry&)> OdometryCallback;
    typedef std::function<int32_t(const MapLanes&)> PointsCallback;

public:

    /// @brief Define shared pointer type name
    typedef std::shared_ptr<IbeoBasedLocalization> Ptr;
    typedef std::shared_ptr<const IbeoBasedLocalization> ConstPtr;

    struct Parameters : public ParametersBase
    {
        /// @brief set parameters
        Parameters();

        /// @brief frequence
        Scalar estimation_frequence_;

        /// @brief Max beams to consider
        size_t max_beams_;

        /// @brief Limit the min particles and max particles were used
        size_t min_particles_, max_particles_;

        /// @brief Drift parameters, rotation and translation noise
        /// @brief Reference to <<Probabilistic Robot>>, chapter 5
        Scalar alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;

        /// @brief Laser model params, mixture params for the components of the model
        Scalar z_hit_, z_rand_, sigma_hit_, lambda_short_;//

        /// @brief Decay rates for running averages
        Scalar alpha_slow_, alpha_fast_;

        /// @brief Population size parameters
        Scalar pf_err_, pf_z_;

        /// @brief Threshold of Motion update
        Scalar delta_thresh_;

        // Scalar x_err_thresh_;
        // Scalar y_err_thresh_;
        // Scalar yaw_err_thresh_;

        /// @brief Threshold of min link length
        Scalar min_lane_length_;


        /// @brief Whether update pose or not
        bool update_pose_;

        /// @brief Observation model
        LaserModelT laser_model_type_;

        /// @brief Motion model
        OdomModelT odom_model_type_;

        /// @brief A parameter of string type for observation model        
        std::string laser_model_type_string_;

        /// @brief A parameter of string type for motion model        
        std::string odom_model_type_string_;

        /// @brief Initial pose covariance
        Scalar initial_cov_xx_;
        Scalar initial_cov_yy_;
        Scalar initial_cov_aa_;
    };

    struct Options
    {
        /// @brief will record evaluation files?
        bool will_record_evaluation_files;

        /// @brief set options
        Options(const bool& _will_record_evaluation_files = false);
    };

    // @brief default constructor
    IbeoBasedLocalization();

    // @brief default constructor
    IbeoBasedLocalization(const Parameters& parameters = Parameters(),const Options& options = Options());

    /// @brief destructor
    virtual ~IbeoBasedLocalization();

    /// @brief process received ibeo scan datas
    /// @param[in] ibeo scan datas
    int32_t ProcessIbeoScanData(const IbeoScanData& received_ibeo_scan_data);

    // @brief process road info
    // @param[in] road_info
    int32_t ProcessRouteInfo(const RouteInfo& route_info);

    /// @brief process ground truth
    /// @param[in] odometry
    int32_t ProcessGroundTruth(const Odometry& odometry);
    
    /// @brief publish optimized odometry
    void PublishOptimizedOdometry();

    /// @brief set output ibeo odometry callback
    /// @param[in] odometry_callback
    void SetIbeoOdometryCallback(const OdometryCallback& odometry_callback);

    /// @brief process prior odometry
    /// @param[in] lane_id
    int32_t ProcessPriorOdometry(const Odometry& odometry);

private:

    /// @brief check pre condition
    virtual bool CheckPreCondition() override;

    /// @brief estimation core function for ibeo-based localization pipeline.
    virtual void EstimationCore() override;

    /// @brief post process
    virtual bool PostProcess() override;

    /// @brief get parameters;
    virtual const Parameters& Params() const override;

    /// @brief got latest prior odometry?
    /// return got prior odometry or not
    bool GotLatestPriorOdometry();

    /// @brief got latest roadside points?
    /// return got latest roadside points or not
    bool GotLatestRoadSideMapPoints();

    bool GotLatestIbeoScanData();

    /// @brief got latest input odometry?
    /// return got input odometry or not
    bool GotLatestInitialOdometry();

    /// @brief Set observation and motion model?
    /// return set or not
    bool SetOdomAndSensorModel();
    
    struct SolutionState
    {
        /// @brief default constructor
        SolutionState();

        /// @brief optimized pose
        Pose3 optimized_pose;

        /// @brief covariance matrix
        Matrix6 covariance;

        /// @brief optimized or not?
        bool optimized;

    }solution_;

    /// @brief parameters of camera, odometry and map
    Parameters parameters_;

    /// @brief options of recording and visualization
    Options options_;

    /// @brief current timestamp of system(ibeo)
    TimeStamp cur_time_;

    /// @brief last timestamp of system(ibeo)
    TimeStamp last_time_;

    /// @brief ground truth odometry
    Odometry ground_truth_odometry_;

    /// @brief unoptimized odometry is used for optimization.
    Odometry unoptimized_odometry_;

    /// @brief a vector for 3D pose's degrees
    PfVectorT yawpitchroll_;

    /// @brief latest ibeo scan data
    IbeoScanData latest_ibeo_scan_data_;

    /// @brief fine odometry accesses from fusion
    Odometry fine_odometry_;

    /// @brief lane boundary points of map lane
    MapPoints map_left_points_;
    MapPoints map_right_points_;

    /// @brief lane measurements manager pointer
    MeasurementsManagerIbeoPtr input_manager_;

    /// @brief class my_amcl shared pointer
    MyAmclPtr my_amcl_ptr_;

    /// @brief whether pf is initialized or not
    bool pf_init_;

    /// @brief Information for an entire filter
    PfT* pf_;

    /// @brief Description for a grid map
    MapT* map_;

    /// @brief Laseretric sensor model
    MyAmcl* laser_;

    /// @brief last pose from odometry
    PfVectorT last_unoptimized_pose_;
    PfVectorT last_optimized_pose_;

    /// @brief localization thread pointer
    std::shared_ptr<std::thread> localization_thread_; 

    /// @brief output callback
    OdometryCallback ibeo_odometry_callback_;
    PointsCallback detected_lane_callback_;
    PointsCallback ground_truth_lane_callback_;
    PointsCallback unoptimized_lane_callback_;
    PointsCallback optimized_lane_callback_;
};

} // ibeo namespace
} // localization namespace
} // my namespace


#endif //IBEO_BASED_LOCALIZATION_H_
