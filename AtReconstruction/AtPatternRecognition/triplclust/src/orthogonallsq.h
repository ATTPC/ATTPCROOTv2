//
// orthogonallsq.h
//     Orthogonal Least Squares Fit by means of PCA.
//
// Author:  Christoph Dalitz
// Date:    2022-11-07
// License: see ../LICENSE
//
/** @file */

#ifndef ORTHOGONALLSQ_H
#define ORTHOGONALLSQ_H

#include "pointcloud.h"

//
// orthogonal least squares fit with libeigen
//   pc: points
//   a, b: line representation as a + t*b
//   RC: largest eigenvalue
//
double orthogonal_lsq(const PointCloud &pc, Point *a, Point *b);

//
// orthogonal distance of Point c to line a + t*b
//   a, b: line representation as a + t*b
//   c: point
// return: double distance
double orthogonal_lsq_distance(Point *a, Point *b, Point *c);

#endif
