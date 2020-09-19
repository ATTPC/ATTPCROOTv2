/*
 * Project:  LevenbergMarquardtLeastSquaresFitting
 *           code obtained from http://sourceforge.net/projects/lmfit/
 * Contents: optimize p to fit CRDC data y(x)
 *           sechs:      p(0)/cosh((log(sqrt(2)+1)/sqrt(2*log(2)))*((x-p(1))/p(2))^2
 *           gaussian:   p(0)*exp(-(x-p(1))^2/(2*p(2)^2))
 *                       height: p(0), centroid: p(1), FWHM: 2*sqrt(2*log(2))*p(3)
 * Author:   Shumpei Noji
 */

#include "lmmin.hh"
#include "lmcurve.hh"
#include "lmfit.hh"
#include <stdio.h>
#include <math.h>

double sechs(double x, const double *p) {
  //  return p[0] / pow( cosh( M_PI * ( (x - p[1]) / p[2] ) ), 2 );
  return p[0] / pow( cosh( (log(sqrt(2)+1)/sqrt(2*log(2))) * ( (x - p[1]) / p[2] ) ), 2 );
}

double gauss(double x, const double *p) {
  return p[0] * exp( - pow( (x - p[1]), 2) / (2 * pow(p[2], 2) ));
}
