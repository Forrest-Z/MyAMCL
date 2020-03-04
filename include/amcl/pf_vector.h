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
 * Desc: Vector functions
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf_vector.h 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#ifndef PF_VECTOR_H
#define PF_VECTOR_H

// #ifdef __cplusplus
// extern "C" {
// #endif

#include <stdio.h>
#include <memory.h>

// The basic vector
struct PfVectorT
{
  double v[3];
  PfVectorT()
  {
    memset(v, 0, sizeof(v));
  }
};


// The basic matrix
struct PfMatrixT
{
  double m[3][3];
} ;


// Return a zero vector
PfVectorT PfVectorZero();

// Check for NAN or INF in any component
int PfVectorFinite(PfVectorT a);

// Print a vector
void PfVectorFprintf(PfVectorT s, FILE *file, const char *fmt);

// Simple vector addition
PfVectorT PfVectorAdd(PfVectorT a, PfVectorT b);

// Simple vector subtraction
PfVectorT PfVectorSub(PfVectorT a, PfVectorT b);

// Transform from local to global coords (a + b)
PfVectorT PfVectorCoordAdd(PfVectorT a, PfVectorT b);

// Transform from global to local coords (a - b)
PfVectorT PfVectorCoordSub(PfVectorT a, PfVectorT b);


// Return a zero matrix
PfMatrixT PfMatrixZero();

// Check for NAN or INF in any component
int PfMatrixFinite(PfMatrixT a);

// Print a matrix
void PfMatrixFprintf(PfMatrixT s, FILE *file, const char *fmt);

// Compute the matrix inverse.  Will also return the determinant,
// which should be checked for underflow (indicated singular matrix).
//PfMatrixT pf_matrix_inverse(PfMatrixT a, double *det);

// Decompose a covariance matrix [a] into a rotation matrix [r] and a
// diagonal matrix [d] such that a = r * d * r^T.
void PfMatrixUnitary(PfMatrixT *r, PfMatrixT *d, PfMatrixT a);

// #ifdef __cplusplus
// }
// #endif

#endif
