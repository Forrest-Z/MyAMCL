/*!
 * * \brief the definition of front-ibeo-based localization class
 * * \author Shifan Zhu (zhushifan12@gmail.com)
 * * \date May 29, 2019
 */


#ifndef MEASUREMENTS_MANAGER_IBEO_LOCALIZATION_H_
#define MEASUREMENTS_MANAGER_IBEO_LOCALIZATION_H_

#include <measurements_manager_base.h>
#include <map/current_route_info.h>
#include <obstacle/obstacle_list.h>
#include <sensors/ibeo/ibeo_structure.h>

namespace my
{
namespace localization
{
namespace ibeo
{

class MeasurementsManagerIbeo : public MeasurementsManagerBase
{

private:

    /// define type name.
    typedef my::map::CurrentRouteInfo RouteInfo;
    typedef boost::circular_buffer<RouteInfo> RouteInfoCircularBuffer;
    typedef boost::circular_buffer<Odometry> OdometryCircularBuffer;
    typedef my::sensors::ibeo::IbeoScanData2202 IbeoScanData;
    typedef boost::circular_buffer<IbeoScanData> IbeoScanDataCircularBuffer;

public:

    typedef MeasurementsManagerBase Base;
    using Base::AddMeasurement;
    using Base::GetLatestMeasurement;

    /// define shared pointer type name.
    typedef std::shared_ptr<MeasurementsManagerIbeo> Ptr;
    typedef std::shared_ptr<const MeasurementsManagerIbeo> ConstPtr;

    /// @brief default constructor
    MeasurementsManagerIbeo();

    /// @brief default destructor
    virtual ~MeasurementsManagerIbeo();

    /// @brief add route info
    /// @param[in] route_info
    bool AddMeasurement(const RouteInfo& route_info);


    bool AddIbeoScanData(const IbeoScanData& received_ibeo_scan_data);

    /// @brief add prior odometry
    /// @param[in] odometry
    bool AddPriorOdometry(const Odometry& odometry);

    /// @brief get current route info
    /// @param[out] route_info 
    bool GetLatestMeasurement(RouteInfo& route_info);

    bool GetLatestIbeoScanData(IbeoScanData& latest_ibeo_scan_data);

    /// @brief get current prior odometry by linear interpolation
    /// @param[in] time
    /// @param[out] odometry
    bool GetCurrentPriorOdometry(const TimeStamp& time, Odometry& odometry);

    /// @brief get current ground truth by linear interpolation
    /// @param[in] time
    /// @param[out] odometry
    bool GetCurrentGroundTruth(const TimeStamp& time, Odometry& odometry);

private:

    /// define type name.
    using Base::Scalar;
    using Base::Rot3;
    using Base::Quaternion;
    using Base::Point3;
    using Base::Pose3;

    /// @brief get current odometry by linear interpolation.
    /// @param[in] time
    /// @param[in] odometries
    /// @param[out] odometry.
    bool GetCurrentOdometry(const TimeStamp& time,
                            const OdometryCircularBuffer& odometries,
                            Odometry& odometry);

    /// define circular buffers;
    RouteInfoCircularBuffer route_info_queue_;
    OdometryCircularBuffer prior_odometry_queue_;
    IbeoScanDataCircularBuffer ibeo_scan_data_queue_;

    /// define mutex;
    std::mutex route_info_mutex_;
    
    // std::mutex init_lane_odometry_mutex_;
    std::mutex prior_odometry_mutex_;
    std::mutex received_ibeo_data_mutex_;
    std::mutex roadside_mappoints_mutex_;

}; // MeasurementManagerLane class

} // ibeo namespace
} // localization namespace
} // my namespace

#endif // MEASUREMENTS_MANAGER_IBEO_LOCALIZATION_H_
