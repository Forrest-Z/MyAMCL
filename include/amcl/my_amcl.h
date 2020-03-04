/*!
 * * \brief the definition of front-ibeo-based localization class
 * * \author Shifan Zhu (zhushifan12@gmail.com)
 * * \date May 29, 2019
 */


#ifndef MY_AMCL_H_
#define MY_AMCL_H_

#include <common/odometry.h>
#include <core/types.h>
#include <geometry/polyfit2.h>
#include <geometry/polyfit3.h>
#include <map/link_info.h>
#include <localization/lidar/ibeo/measurements_manager_ibeo.h>
#include "localization/lidar/ibeo/amcl/pf.h"

namespace my
{
namespace localization
{
namespace ibeo
{

typedef enum
{
  ODOM_MODEL_OMNI,
  ODOM_MODEL_DIFF
} OdomModelT;

typedef enum
{
  LASER_MODEL_LIKELIHOOD_FIELD = 0,
  LASER_MODEL_DISTANCE = 1
} LaserModelT;

// Description for a single map cell.
struct MapCellT
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int occ_state;

  // Distance to the nearest occupied cell
  double occ_dist;

  // Wifi levels
  //int wifi_levels[MAP_WIFI_MAX_LEVELS];

};

// Description for a map
struct MapT
{
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;
  
  // Map scale (m/cell)
  double scale;

  // Map dimensions (number of cells)
  int size_x, size_y;
  
  // The map data, stored as a grid
  MapCellT *cells;

  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;
  
};


// Forward declarations
class AmclSensorData;

class AmclSensor
{
public: 
    AmclSensor(){};
    virtual ~AmclSensor(){};

    /// @brief update the filter based on the action model.
    /// @param[in] particles
    /// @param[in] odom data
    /// Returns true if the filter has been updated.
    virtual bool UpdateAction(PfT *pf, AmclSensorData *data);

    /// @brief update the filter based on the sensor model.
    /// @param[in] particles
    /// @param[in] sensor data, e.g. lidar data
    /// @param[in] left roadside lane
    /// @param[in] right roadside lane
    /// Returns true if the filter has been updated.
    virtual ObservationResult UpdateSensor(PfT *pf, AmclSensorData *data, MapPoints& map_left_points_, MapPoints& map_right_points_);

    /// @brief Action pose
    PfVectorT pose;
};

/// @brief Base class for all AMCL sensor measurements
class AmclSensorData
{
public: 
    /// @brief Pointer to sensor that generated the data
    AmclSensor *sensor;

    /// @brief Default destructor
    virtual ~AmclSensorData() {}

    /// @brief Data timestamp
    Scalar time_;
};

class AmclOdomData : public AmclSensorData
{
    /// @brief Odometric pose
    public: PfVectorT pose_;

    /// @brief Change in odometric pose
    public: PfVectorT delta_;
};

class AmclLaserData : public AmclSensorData
{
private: 
    typedef my::float64_t Scalar;

public:
    AmclLaserData () {ranges=NULL;};
    virtual ~AmclLaserData() {delete [] ranges;};

    /// @brief Laser range data (range, bearing tuples)
    int32_t range_count_;
    Scalar range_max_;
    Scalar (*ranges)[2];
};

class MyAmcl : public AmclSensor
{
private:
    typedef my::Odometry Odometry;
    typedef my::sensors::ibeo::IbeoScanData2202 IbeoScanData;
    typedef my::float64_t Scalar;
    typedef my::Point3T<Scalar> Point3;
    typedef Point2T<Scalar>   Point2Type;
    typedef Point3T<Scalar>   Point3Type;
    typedef my::Point3T<Scalar> MapPoint;
    typedef PolyFit3T<Scalar> PolyFit3Type;
    typedef PolyFit2T<Scalar> PolyFit2Type;
    typedef my::map::CurrentRouteInfo RouteInfo;
    typedef my::map::LaneInfo LaneInfo;
    typedef my::map::LinkInfo LinkInfo;
    typedef std::vector<LinkInfo> LinkInfos;
    typedef std::vector<LaneInfo> LaneInfos;
    typedef my::map::LaneMarkInfo LaneMarkInfo;
    typedef std::vector<Point3> MapPoints;
    typedef MeasurementsManagerIbeo::Ptr MeasurementsManagerIbeoPtr;

public:
    typedef std::shared_ptr<MyAmcl> Ptr;
    MyAmcl();
    MyAmcl(uint16_t max_beam, MapT* map);
    ~MyAmcl();

    /// @brief update the filter based on the action model.
    /// @param[in] particles
    /// @param[in] odom data
    /// Returns true if the filter has been updated.
    virtual bool UpdateAction(PfT *pf, AmclSensorData *data);//Odometry * odom);

    /// @brief update the filter based on the sensor model.
    /// @param[in] particles
    /// @param[in] sensor data, e.g. lidar data
    /// @param[in] left roadside lane
    /// @param[in] right roadside lane
    /// Returns true if the filter has been updated.
    virtual ObservationResult UpdateSensor(PfT *pf, AmclSensorData *data, MapPoints& map_left_points_, MapPoints& map_right_points_);

    /// @brief Set differential Wheel Model
    void SetOdomModelDiff(Scalar alpha1,
                          Scalar alpha2,
                          Scalar alpha3,
                          Scalar alpha4);

    /// @brief Set omnidirectional wheel model
    void SetOdomModelOmni(Scalar alpha1,
                          Scalar alpha2,
                          Scalar alpha3,
                          Scalar alpha4,
                          Scalar alpha5);

    /// @brief Set odom model
    void SetOdomModel(OdomModelT type,
                          Scalar alpha1,
                          Scalar alpha2,
                          Scalar alpha3,
                          Scalar alpha4,
                          Scalar alpha5 = 0);

    /// @brief Set likelihood field model
    /// @brief reference to <<Probabilistic Robot>>, chapter 6
    void SetModelLikelihoodField(Scalar z_hit,
                                 Scalar z_rand,
                                 Scalar sigma_hit);

    /// @brief Set Distance model
    void SetModelDistance(Scalar z_hit,
                          Scalar z_rand,
                          Scalar sigma_hit);

    /// @brief Set laser pose(from laser frame to base frame)
    void SetLaserPose(PfVectorT& laser_pose)
    {this->laser_pose_ = laser_pose;}

    /// @brief Get angle difference between a and b
    Scalar AngleDiff(Scalar a, Scalar b);

    /// @brief Pose-generating function used to uniformly distribute
    /// particles over the map
    static PfVectorT UniformPoseGenerator(void* arg);

private: 
    /// @brief Drift parameters, rotation and translation noise
    /// @brief Reference to <<Probabilistic Robot>>, chapter 5
    Scalar alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;

    /// @brief Mixture params for the components of the model
    Scalar z_hit_, z_rand_, sigma_hit_;

    /// @brief Max beams to consider
    uint16_t max_beams_;

    /// @brief laser pose(from ibeo to body)
    PfVectorT laser_pose_;

    /// @brief Observation model
    LaserModelT laser_model_type_;

    /// @brief Motion model
    OdomModelT odom_model_type_;
    // PfVectorT pose_;
    // PfVectorT delta_;

    /// @brief Odom data
    AmclOdomData odomdata_;

    MapT* map_;

    Scalar time_;

    /// @brief two ways of calculating costs
    static Scalar LikelihoodFieldModel(); //TODO finish likelihoodfield model
    static ObservationResult DistanceModel(AmclLaserData *data, PfSampleSetT* set, MapPoints& map_left_points_, MapPoints& map_right_points_);

    /// @brief calculate atan2
    Scalar Normalize(Scalar z);

    static std::vector<std::pair<int16_t,int16_t> > free_space_indices;
};

} // ibeo namespace 
} // localization namespace
} // my namespace

#endif // MY_AMCL_H_
