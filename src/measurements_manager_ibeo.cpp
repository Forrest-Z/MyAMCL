/*!
 * * \brief the definition of front-ibeo-based localization class
 * * \author Shifan Zhu (zhushifan12@gmail.com)
 * * \date May 29, 2019
 */


#include <measurements_manager_ibeo.h>

namespace my
{
namespace localization
{
namespace ibeo
{

////////////////////////////////////////////////////////////////////////////////
MeasurementsManagerIbeo::MeasurementsManagerIbeo()
    : Base(ParametersBase()),H
      route_info_queue_(2000),
      prior_odometry_queue_(2000),
      ibeo_scan_data_queue_(2000)
{

}

////////////////////////////////////////////////////////////////////////////////
MeasurementsManagerIbeo::~MeasurementsManagerIbeo()
{

}

////////////////////////////////////////////////////////////////////////////////
bool MeasurementsManagerIbeo::AddMeasurement(const RouteInfo& route_info)
{
    std::unique_lock<std::mutex> lock(route_info_mutex_);

    if (route_info_queue_.empty())
    {
        route_info_queue_.push_back(route_info);
    }
    else if (route_info.GetHeader().stamp() ==
             route_info_queue_.back().GetHeader().stamp())
    {
    }
    else
    {
        route_info_queue_.push_back(route_info);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MeasurementsManagerIbeo::AddIbeoScanData(const IbeoScanData& received_ibeo_scan_data)
{
    std::unique_lock<std::mutex> lock(received_ibeo_data_mutex_);

    ibeo_scan_data_queue_.push_back(received_ibeo_scan_data);

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MeasurementsManagerIbeo::GetLatestMeasurement(RouteInfo& route_info)
{
    std::unique_lock<std::mutex> lock(route_info_mutex_);

    if (route_info_queue_.empty())
    {
        DLOG(WARNING) << "The route info queue is empty.";
        return false;
    }

    route_info = route_info_queue_.back();
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MeasurementsManagerIbeo::GetLatestIbeoScanData(IbeoScanData& latest_ibeo_scan_data)
{
    std::unique_lock<std::mutex> lock(received_ibeo_data_mutex_);

    if(ibeo_scan_data_queue_.empty())
    {
        DLOG(WARNING) << "The ibeo scan data queue is empty.";
        return false;
    }

    latest_ibeo_scan_data = ibeo_scan_data_queue_.back();
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MeasurementsManagerIbeo::GetCurrentPriorOdometry(const TimeStamp& time,
        Odometry& odometry)
{
    std::unique_lock<std::mutex> lock(prior_odometry_mutex_);

    if (!GetCurrentOdometry(time, prior_odometry_queue_, odometry))
    {
        LOG(WARNING) << "The prior odometry queue is empty.";
        return false;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MeasurementsManagerIbeo::GetCurrentGroundTruth(
    const TimeStamp& time, Odometry& odometry)
{
    // std::unique_lock<std::mutex> lock(ground_truth_mutex_);

    if (!GetCurrentOdometry(time, ground_truth_queue_, odometry))
    {
        LOG(WARNING) << "The ground truth odometry queue is empty.";
        return false;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MeasurementsManagerIbeo::
GetCurrentOdometry(const TimeStamp& time,
                   const OdometryCircularBuffer& odometrie_queue,
                   Odometry& odometry)
{
    if (odometrie_queue.size() < 2)
    {
        LOG(ERROR) << "Data is not sufficient.";
        return false;
    }

    Odometry odometry_0, odometry_1;
    LOG(INFO) << "time = " << time;
    LOG(INFO) << "odometrie_queue.back().header().stamp() = "<<odometrie_queue.back().header().stamp();

    if (time <= odometrie_queue.front().header().stamp())
    {
        odometry_0 = *(odometrie_queue.begin());
        odometry_1 = *(odometrie_queue.begin() + 1);

        LOG(WARNING) << "Query time is lagging.";
    }
    else if (time >= odometrie_queue.back().header().stamp())
    {
        odometry_1 = *(odometrie_queue.rbegin());
        odometry_0 = *(odometrie_queue.rbegin() + 1);

        LOG(WARNING) << "Query time is leading.";
    }
    else
    {
        // not less than
        auto it = (std::lower_bound(odometrie_queue.begin(), odometrie_queue.end(),
                                    time, StampedTimeCompare<Odometry>()));
        odometry_0 = *(it - 1);
        odometry_1 = *it;
    }

    Scalar time_0 = odometry_0.header().stamp().ToSec();
    Scalar time_1 = odometry_1.header().stamp().ToSec();
    Scalar time_range = time_1 - time_0;

    if (std::fabs(time_range) < 1E-6)
    {
        LOG(ERROR) << "The timestamps are consistent.";
        return false;
    }

    // time range threshold to determine whether should we do interpolate or not
    const Scalar time_range_threshold = 1.0;

    if (std::fabs(time_range) > time_range_threshold ||
            std::min(std::fabs(time.ToSec() - time_0),
                     std::fabs(time.ToSec() - time_1)) > time_range_threshold)
    {
        LOG(ERROR) << "time range exceeds the threshold !";
        return false;
    }

    odometry = odometry_0;
    Scalar ratio = (time.ToSec() - time_0) / (time_1 - time_0);

    Quaternion rotation_0 = odometry_0.pose().Rotation().Quaternion();
    Quaternion rotation_1 = odometry_1.pose().Rotation().Quaternion();
    Rot3 rotation = Rot3(rotation_0.slerp(ratio, rotation_1).toRotationMatrix());

    Point3 position_0 = odometry_0.pose().Translation();
    Point3 position_1 = odometry_1.pose().Translation();
    Point3 position = (1.0 - ratio) * position_0 + ratio * position_1;

    Pose3 pose(rotation, position);
    odometry.pose() = pose;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MeasurementsManagerIbeo::AddPriorOdometry(const Odometry& odometry)
{
    return InsertMeasurement(odometry, prior_odometry_mutex_, prior_odometry_queue_);
}

} // ibeo namespace
} // localization namespace
} // my namespace
