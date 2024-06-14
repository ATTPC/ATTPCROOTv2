//
// orthogonallsq.h
//     Orthogonal Least Squares Fit by means of PCA.
//
// Author:  Christoph Dalitz
// Date:    2022-11-07
// License: see ../LICENSE
//
/** @file */

#include "pointcloud.h"

#include <eigen3/Eigen/Dense>
using Eigen::MatrixXf;
using Eigen::Vector3f;

//
// orthogonal least squares fit with libeigen
//   pc: points
//   a, b: line representation as a + t*b
//   RC: largest eigenvalue
//
double orthogonal_lsq(const PointCloud &pc, Point *a, Point *b)
{
   double rc = 0.0;

   if (pc.size() == 0)
      return rc;

   // anchor point is mean value
   a->x = a->y = a->z = 0.0;
   for (size_t i = 0; i < pc.size(); i++) {
      *a = *a + pc[i];
   }
   *a = *a / pc.size();

   // copy points to libeigen matrix
   Eigen::MatrixXf points = Eigen::MatrixXf::Constant(pc.size(), 3, 0);
   for (unsigned int i = 0; i < points.rows(); i++) {
      points(i, 0) = pc[i].x;
      points(i, 1) = pc[i].y;
      points(i, 2) = pc[i].z;
   }

   // compute scatter matrix ...
   MatrixXf centered = points.rowwise() - points.colwise().mean();
   MatrixXf scatter = (centered.adjoint() * centered);

   // ... and its eigenvalues and eigenvectors
   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(scatter);
   Eigen::MatrixXf eigvecs = eig.eigenvectors();

   // we need eigenvector to largest eigenvalue
   // libeigen yields it as LAST column
   b->x = eigvecs(0, 2);
   b->y = eigvecs(1, 2);
   b->z = eigvecs(2, 2);
   rc = eig.eigenvalues()(2);

   return (rc);
}

double orthogonal_lsq_distance(Point *a, Point *b, Point *c)
{
   double distance = 0;
   double lambda;

   // copy points to libeigen vektor
   Eigen::Vector3f a_vec = Eigen::Vector3f(a->x, a->y, a->z);
   Eigen::Vector3f b_vec = Eigen::Vector3f(b->x, b->y, b->z);
   Eigen::Vector3f c_vec = Eigen::Vector3f(c->x, c->y, c->z);

   lambda = b_vec.transpose() * (c_vec - a_vec);

   distance = (c_vec - (a_vec + lambda * b_vec)).norm();

   return distance;
}