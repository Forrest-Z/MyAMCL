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
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <time.h>

#include "my/localization/lidar/ibeo/amcl/pf.h"
#include "my/localization/lidar/ibeo/amcl/pf_pdf.h"
#include "my/localization/lidar/ibeo/amcl/pf_kdtree.h"
#include "my/localization/lidar/ibeo/amcl/portable_utils.hpp"


// Compute the required number of samples, given that there are k bins
// with samples in them.
static int PfResampleLimit(PfT *pf, int k);



// Create a new filter
PfT *PfAlloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               PfInitModelFnT random_pose_fn, void *random_pose_data)
{
  int i, j;
  PfT *pf;
  PfSampleSetT *set;
  PfSampleT *sample;
  
  Srand48(time(NULL));

  pf = (PfT*) calloc(1, sizeof(PfT));

  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5; 
  
  pf->current_set = 0;
  for (j = 0; j < 2; j++)
  {
    set = pf->sets + j;
      
    set->sample_count = max_samples;
    set->samples = (PfSampleT*) calloc(max_samples, sizeof(PfSampleT));

    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    set->kdtree = PfKdtreeAlloc(3 * max_samples);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = (PfClusterT*) calloc(set->cluster_max_count, sizeof(PfClusterT));

    set->mean = PfVectorZero();
    set->cov = PfMatrixZero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  //set converged to 0
  PfInitConverged(pf);

  return pf;
}

// Free an existing filter
void PfFree(PfT *pf)
{
  int i;
  
  for (i = 0; i < 2; i++)
  {
    free(pf->sets[i].clusters);
    PfKdtreeFree(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);
  
  return;
}

// Initialize the filter using a guassian
void PfInit(PfT *pf, PfVectorT mean, PfMatrixT cov)
{
  LOG(INFO) << "In function PfInit";
  int i;
  PfSampleSetT *set;
  PfSampleT *sample;
  PfPdfGaussianT *pdf;
  
  set = pf->sets + pf->current_set;
  
  // Create the kd tree for adaptive sampling
  PfKdtreeClear(set->kdtree);

  set->sample_count = pf->max_samples;

  pdf = PfPdfGaussianAlloc(mean, cov);
    
  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = PfPdfGaussianSample(pdf);

    // Add sample to histogram
    PfKdtreeInsert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  PfPdfGaussianFree(pdf);
    
  // Re-compute cluster statistics
  PfClusterStats(set); 

  //set converged to 0
  PfInitConverged(pf);

  return;
}


// Initialize the filter using some model
void PfInitModel(PfT *pf, PfInitModelFnT init_fn, void *init_data)
{
  int i;
  PfSampleSetT *set;
  PfSampleT *sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  PfKdtreeClear(set->kdtree);

  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = (*init_fn) (init_data);

    // Add sample to histogram
    PfKdtreeInsert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  PfClusterStats(set);
  
  //set converged to 0
  PfInitConverged(pf);

  return;
}

void PfInitConverged(PfT *pf){
  PfSampleSetT *set;
  set = pf->sets + pf->current_set;
  set->converged = 0; 
  pf->converged = 0; 
}

int PfUpdateConverged(PfT *pf)
{
  int i;
  PfSampleSetT *set;
  PfSampleT *sample;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;
  
  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;
    if(fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold || 
       fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold){
      set->converged = 0; 
      pf->converged = 0; 
      return 0;
    }
  }
  set->converged = 1; 
  pf->converged = 1; 
  return 1; 
}

// Update the filter with some new action
void PfUpdateAction(PfT *pf, PfActionModelFnT action_fn, void *action_data)
{
  PfSampleSetT *set;

  set = pf->sets + pf->current_set;

  (*action_fn) (action_data, set);
  
  return;
}


// Update the filter with some new sensor observation
ObservationResult PfUpdateSensor(PfT *pf, PfSensorModelFnT sensor_fn, void *sensor_data, MapPoints& map_left_points_, MapPoints& map_right_points_)
// void PfUpdateSensor(PfT *pf, PfSensorModelFnT sensor_fn, void *sensor_data)
{
  int i;
  PfSampleSetT *set;
  PfSampleT *sample;
  // Scalar total;
  ObservationResult obs_res;

  set = pf->sets + pf->current_set;

  // Compute the sample weights
  obs_res = (*sensor_fn) (sensor_data, set, map_left_points_, map_right_points_);
  // total = (*sensor_fn) (sensor_data, set, map_left_points_, map_right_points_);
  
  if (obs_res.total_weight_  > 0.0)
  {
    // Normalize weights
    double w_avg=0.0;
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      w_avg += sample->weight;
      sample->weight /= obs_res.total_weight_;
    }
    // Update running averages of likelihood of samples (Prob Rob p258)
    w_avg /= set->sample_count;
    if(pf->w_slow == 0.0)
      pf->w_slow = w_avg;
    else
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
    if(pf->w_fast == 0.0)
      pf->w_fast = w_avg;
    else
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    //printf("w_avg: %e slow: %e fast: %e\n", 
           //w_avg, pf->w_slow, pf->w_fast);
  }
  else
  {
    // Handle zero total
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }

  return obs_res;
}


// Resample the distribution
void PfUpdateResample(PfT *pf)
{
  int i;
  double total;
  PfSampleSetT *set_a, *set_b;
  PfSampleT *sample_a = new PfSampleT;
  PfSampleT *sample_b;
  double* c;

  double w_diff;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  c = (double*)malloc(sizeof(double)*(set_a->sample_count+1));
  c[0] = 0.0;
  for(i=0;i<set_a->sample_count;i++)
    c[i+1] = c[i]+set_a->samples[i].weight;

  // Create the kd tree for adaptive sampling
  PfKdtreeClear(set_b->kdtree);
  
  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;

  w_diff = 1.0 - pf->w_fast / pf->w_slow;
  if(w_diff < 0.0)
    w_diff = 0.0;

  while(set_b->sample_count < pf->max_samples)
  {
    sample_b = set_b->samples + set_b->sample_count++;

    if(Drand48() < w_diff)
      sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
    else
    {
      // Naive discrete event sampler
      double r;
      r = Drand48();
      for(i=0;i<set_a->sample_count;i++)
      {
        if((c[i] <= r) && (r < c[i+1]))
          break;
      }
      assert(i<set_a->sample_count);

      sample_a = set_a->samples + i;

      assert(sample_a->weight > 0);

      // Add sample to list
      sample_b->pose = sample_a->pose;
    }

    // sample_b->weight = 1.0;//original_code
    sample_b->weight = sample_a->weight;
    total += sample_b->weight;

    // Add sample to histogram
    PfKdtreeInsert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    if (set_b->sample_count > PfResampleLimit(pf, set_b->kdtree->leaf_count))
      break;
  }
  
  // Reset averages, to avoid spiraling off into complete randomness.
  if(w_diff > 0.0)
    pf->w_slow = pf->w_fast = 0.0;
  
  // Re-compute cluster statistics
  PfClusterStats(set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2; 

  PfUpdateConverged(pf);

  free(c);
  return;
}

// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int PfResampleLimit(PfT *pf, int k)
{
  double a, b, c, x;
  int n;

  if (k <= 1)
    return pf->max_samples;

  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  if (n < pf->min_samples)
    return pf->min_samples;
  if (n > pf->max_samples)
    return pf->max_samples;
  
  return n;
}

/// bubble sort: sort by weight from high to low
void BubbleSort(PfSampleSetT *set, int n)
{
  int i,j;
  for(j = 0; j < n -1; j++)
  {
    for(i = 0; i < n - 1 - j; i++)
    {
      PfSampleT* sample1;
      PfSampleT* sample2;
      double temp_weight;
      PfVectorT temp_pose;
      sample1 = set->samples + i;
      sample2 = set->samples + i + 1;
      if(sample1->weight < sample2->weight)
      {
        temp_weight = sample1->weight;
        temp_pose = sample1->pose;
        sample1->weight = sample2->weight;
        sample1->pose = sample2->pose;
        sample2->weight = temp_weight;
        sample2->pose = temp_pose;
      }
    }
  }
}

// Re-compute the cluster statistics for a sample set
void PfClusterStats(PfSampleSetT *set)
{
  int i, j, k, cidx;
  PfSampleT *sample;
  PfClusterT *cluster;

  //Here is the code I added
  PfSampleSetT *set_sort;///added
  set_sort = set;
  double total_weight = 0.0;
  BubbleSort(set_sort, set_sort->sample_count);
  PfVectorT average_pose;
  average_pose.v[0] = 0.0;
  average_pose.v[1] = 0.0;
  average_pose.v[2] = 0.0;
  for(int cnt = 0;cnt < set_sort->sample_count / 20; cnt++)
  {
    total_weight += (set_sort->samples+cnt)->weight;
  }
  for(int cnt = 0;cnt < set_sort->sample_count / 20; cnt++)
  {
    (set_sort->samples+cnt)->weight /= total_weight;
    average_pose.v[0] += (set_sort->samples+cnt)->pose.v[0] * (set_sort->samples+cnt)->weight;
    average_pose.v[1] += (set_sort->samples+cnt)->pose.v[1] * (set_sort->samples+cnt)->weight;
    average_pose.v[2] += (set_sort->samples+cnt)->pose.v[2] * (set_sort->samples+cnt)->weight;
  }

  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  PfKdtreeCluster(set->kdtree);
  
  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++)
  {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = PfVectorZero();
    cluster->cov = PfMatrixZero();

    for (j = 0; j < 4; j++)
      cluster->m[j] = 0.0;
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->c[j][k] = 0.0;
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = PfVectorZero();
  set->cov = PfMatrixZero();
  for (j = 0; j < 4; j++)
    m[j] = 0.0;
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      c[j][k] = 0.0;
  
  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    // Get the cluster label for this sample
    cidx = PfKdtreeGetCluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count)
      continue;
    if (cidx + 1 > set->cluster_count)
      set->cluster_count = cidx + 1;
    
    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
      {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++)
  {
    cluster = set->clusters + i;
        
    // cluster->mean.v[0] = cluster->m[0] / cluster->weight;//original code
    // cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    // cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);
    cluster->mean.v[0] = average_pose.v[0];
    cluster->mean.v[1] = average_pose.v[1];
    cluster->mean.v[2] = average_pose.v[2];

    cluster->cov = PfMatrixZero();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                         cluster->m[3] * cluster->m[3]));

    cluster->weight = cluster->weight / cluster->count;///added
  }

  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

  return;
}


// Compute the CEP statistics (mean and variance).
void PfGetCepStats(PfT *pf, PfVectorT *mean, double *var)
{
  int i;
  double mn, mx, my, mrr;
  PfSampleSetT *set;
  PfSampleT *sample;
  
  set = pf->sets + pf->current_set;

  mn = 0.0;
  mx = 0.0;
  my = 0.0;
  mrr = 0.0;
  
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    mn += sample->weight;
    mx += sample->weight * sample->pose.v[0];
    my += sample->weight * sample->pose.v[1];
    mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
    mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
  }

  mean->v[0] = mx / mn;
  mean->v[1] = my / mn;
  mean->v[2] = 0.0;

  *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

  return;
}


// Get the statistics for a particular cluster.
int PfGetClusterStats(PfT *pf, int clabel, double *weight,
                         PfVectorT *mean, PfMatrixT *cov)
{
  PfSampleSetT *set;
  PfClusterT *cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count)
    return 0;
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}


