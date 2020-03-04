/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.h 3293 2005-11-19 08:37:45Z gerkey $
 *************************************************************************/

#ifndef PF_H
#define PF_H

#include "pf_vector.h"
#include "pf_kdtree.h"
#include "pf_pdf.h"

#include <holo/common/odometry.h>

// Forward declarations
struct PfT;
struct _rtk_fig_t;
struct PfSampleSetT;

typedef holo::float64_t Scalar;
typedef holo::Point3T<Scalar> Point3;
typedef std::vector<Point3> MapPoints;

// Function prototype for the initialization model; generates a sample pose from
// an appropriate distribution.
typedef PfVectorT (*PfInitModelFnT) (void *init_data);

// Function prototype for the action model; generates a sample pose from
// an appropriate distribution
typedef void (*PfActionModelFnT) (void *action_data, 
                                      struct PfSampleSetT* set);

struct ObservationResult
{
  Scalar total_weight_;
  Scalar observation_dist_;
  ObservationResult()
  {
    total_weight_ = 0;
    observation_dist_ = 100;
  };
};

// Function prototype for the sensor model; determines the probability
// for the given set of sample poses.
// typedef Scalar (*PfSensorModelFnT) (void *sensor_data, struct PfSampleSetT* set);
typedef ObservationResult (*PfSensorModelFnT) (void *sensor_data, struct PfSampleSetT* set,
                          MapPoints& map_left_points_, MapPoints& map_right_points_);

// Information for a single sample
struct PfSampleT
{
  // Pose represented by this sample
  PfVectorT pose;

  // Weight for this pose
  double weight;
  
};


// Information for a cluster of samples
struct PfClusterT
{
  // Number of samples
  int count;

  // Total weight of samples in this cluster
  double weight;

  // Cluster statistics
  PfVectorT mean;
  PfMatrixT cov;

  // Workspace
  double m[4], c[2][2];
  
};


// Information for a set of samples
struct PfSampleSetT
{
  // The samples
  int sample_count;
  PfSampleT *samples;

  // A kdtree encoding the histogram
  PfKdtreeT *kdtree;

  // Clusters
  int cluster_count, cluster_max_count;
  PfClusterT *clusters;

  // Filter statistics
  PfVectorT mean;
  PfMatrixT cov;
  int converged; 
};


// Information for an entire filter
struct PfT
{
  // This min and max number of samples
  int min_samples, max_samples;

  // Population size parameters
  double pop_err, pop_z;
  
  // The sample sets.  We keep two sets and use [current_set]
  // to identify the active set.
  int current_set;
  PfSampleSetT sets[2];

  // Running averages, slow and fast, of likelihood
  double w_slow, w_fast;

  // Decay rates for running averages
  double alpha_slow, alpha_fast;

  // Function used to draw random pose samples
  PfInitModelFnT random_pose_fn;
  void *random_pose_data;

  double dist_threshold; //distance threshold in each axis over which the pf is considered to not be converged
  int converged; 
};

struct AmclHypT
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  PfVectorT pf_pose_mean;

  // Covariance of pose estimate
  PfMatrixT pf_pose_cov;
};

// Create a new filter
PfT *PfAlloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               PfInitModelFnT random_pose_fn, void *random_pose_data);

// Free an existing filter
void PfFree(PfT *pf);

// Initialize the filter using a guassian
void PfInit(PfT *pf, PfVectorT mean, PfMatrixT cov);

// Initialize the filter using some model
void PfInitModel(PfT *pf, PfInitModelFnT init_fn, void *init_data);

// Update the filter with some new action
void PfUpdateAction(PfT *pf, PfActionModelFnT action_fn, void *action_data);

// Update the filter with some new sensor observation
// void PfUpdateSensor(PfT *pf, PfSensorModelFnT sensor_fn, void *sensor_data);
ObservationResult PfUpdateSensor(PfT *pf, PfSensorModelFnT sensor_fn, void *sensor_data, MapPoints& map_left_points_, MapPoints& map_right_points_);

// Resample the distribution
void PfUpdateResample(PfT *pf);

// Compute the CEP statistics (mean and variance).
void PfGetCepStats(PfT *pf, PfVectorT *mean, double *var);

// Compute the statistics for a particular cluster.  Returns 0 if
// there is no such cluster.
int PfGetClusterStats(PfT *pf, int cluster, double *weight,
                         PfVectorT *mean, PfMatrixT *cov);

// Re-compute the cluster statistics for a sample set
void PfClusterStats(PfSampleSetT *set);

//calculate if the particle filter has converged - 
//and sets the converged flag in the current set and the pf 
int PfUpdateConverged(PfT *pf);

//sets the current set and pf converged values to zero
void PfInitConverged(PfT *pf);

// bubble sort by weight from high to low
void BubbleSort(PfSampleSetT *set, int n);

#endif
