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
 * Desc: KD tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.h 6532 2008-06-11 02:45:56Z gbiggs $
 *************************************************************************/

#ifndef PF_KDTREE_H
#define PF_KDTREE_H

// #ifdef INCLUDE_RTKGUI
// #include "rtk.h"
// #endif


// Info for a node in the tree
struct PfKdtreeNodeT
{
  // Depth in the tree
  int leaf, depth;

  // Pivot dimension and value
  int pivot_dim;
  double pivot_value;

  // The key for this node
  int key[3];

  // The value for this node
  double value;

  // The cluster label (leaf nodes)
  int cluster;

  // Child nodes
  struct PfKdtreeNodeT *children[2];

};


// A kd tree
struct PfKdtreeT
{
  // Cell size
  double size[3];

  // The root node of the tree
  PfKdtreeNodeT *root;

  // The number of nodes in the tree
  int node_count, node_max_count;
  PfKdtreeNodeT *nodes;

  // The number of leaf nodes in the tree
  int leaf_count;

};


// Create a tree
extern PfKdtreeT *PfKdtreeAlloc(int max_size);

// Destroy a tree
extern void PfKdtreeFree(PfKdtreeT *self);

// Clear all entries from the tree
extern void PfKdtreeClear(PfKdtreeT *self);

// Insert a pose into the tree
extern void PfKdtreeInsert(PfKdtreeT *self, PfVectorT pose, double value);

// Cluster the leaves in the tree
extern void PfKdtreeCluster(PfKdtreeT *self);

// Determine the probability estimate for the given pose
extern double PfKdtreeGetProb(PfKdtreeT *self, PfVectorT pose);

// Determine the cluster label for the given pose
extern int PfKdtreeGetCluster(PfKdtreeT *self, PfVectorT pose);


// #ifdef INCLUDE_RTKGUI

// // Draw the tree
// extern void pf_kdtree_draw(PfKdtreeT *self, rtk_fig_t *fig);

// #endif

#endif
