/*
 * Project:  LevenbergMarquardtLeastSquaresFitting
 *           code obtained from http://sourceforge.net/projects/lmfit/
 * Contents: optimize p to fit CRDC data y(x)
 *           sechs:      p(0)/cosh((log(sqrt(2)+1)/sqrt(2*log(2)))*((x-p(1))/p(2))^2
 *           gaussian:   p(0)*exp(-(x-p(1))^2/(2*p(2)^2))
 *                       height: p(0), centroid: p(1), FWHM: 2*sqrt(2*log(2))*p(3)
 * Author:   Shumpei Noji
 */

#ifndef LMFIT_H
#define LMFIT_H

#ifdef __cplusplus
extern "C" {
#endif

double sechs(double x, const double *p);
double gauss(double x, const double *p);

#ifdef __cplusplus
}
#endif

#endif /* LMFIT_H */
