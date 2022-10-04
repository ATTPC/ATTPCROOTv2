#include "AtPatternLine.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include <FairLogger.h>

#include <Math/Vector3D.h> // for DisplacementVector3D, operator*
#include <TEveLine.h>
#include <TMath.h>

#include <algorithm> // for min
#include <cmath>     // for cos, sin, pow, sqrt, acos, atan, fabs

class TEveElement;

using namespace AtPatterns;

ClassImp(AtPatternLine);

AtPatternLine::AtPatternLine() : AtPattern(2) {}

TEveElement *AtPatternLine::GetEveElement() const
{
   return AtPattern::GetEveLine(0, 1000, 100);
}
/**
 * @brief Get the parameter closes to compPoint.
 *
 * The clossest point on the line to compPoint is then GetPointAt(return value)
 * @param[in] The point to compare the line to
 * @return The parameter of the point along the line closest to compPoint
 */
double AtPatternLine::parameterAtPoint(const XYZPoint &compPoint) const
{
   return (compPoint - GetPoint()).Dot(GetDirection()) / GetDirection().Mag2();
}

AtPatternLine::XYZPoint AtPatternLine::ClosestPointOnPattern(const XYZPoint &point) const
{
   auto t = parameterAtPoint(point);
   return GetPoint() + t * GetDirection();
}

Double_t AtPatternLine::DistanceToPattern(const XYZPoint &point) const
{
   // Use this rather than parameterAtPoint because it is faster
   auto vec = GetPoint() - point;
   auto nD = GetDirection().Cross(vec);
   double dist2 = nD.Mag2() / GetDirection().Mag2();

   return std::sqrt(dist2);
}

void AtPatternLine::DefinePattern(const std::vector<XYZPoint> &points)
{
   if (points.size() != fNumPoints)
      LOG(error) << "Trying to create pattern with wrong number of points " << points.size();

   auto fPoint = points[0];
   auto fDirection = points[1] - points[0];
   if (fDirection.Z() != 0)
      fDirection /= fabs(fDirection.Z());

   fPatternPar = {fPoint.X(), fPoint.Y(), fPoint.Z(), fDirection.X(), fDirection.Y(), fDirection.Z()};
}

/**
 * @brief Get point on line at z
 *
 * Get point on line at z. If the line is parallel to Z, then return then the parameter passed
 * has not defined physical interpretation
 *
 * @param[in] z Location of point at z [mm]
 */
AtPatternLine::XYZPoint AtPatternLine::GetPointAt(double z) const
{
   return GetPoint() + z * GetDirection();
}

void AtPatternLine::FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge)
{
   //------3D Line Regression
   //----- adapted from: http://fr.scribd.com/doc/31477970/Regressions-et-trajectoires-3D
   int R, C;
   double Q;
   double Xm, Ym, Zm;
   double Xh, Yh, Zh;
   double a, b;
   double Sxx, Sxy, Syy, Sxz, Szz, Syz;
   double theta;
   double K11, K22, K12, K10, K01, K00;
   double c0, c1, c2;
   double p, q, r, dm2;
   double rho, phi;

   Q = Xm = Ym = Zm = 0.;
   double total_charge = 0;
   Sxx = Syy = Szz = Sxy = Sxz = Syz = 0.;
   bool doChargeWeight = points.size() == charge.size();

   for (int i = 0; i < points.size(); ++i) {
      const auto hitQ = doChargeWeight ? charge[i] : 1;
      const auto &pos = points[i];
      Q += hitQ / 10.;
      Xm += pos.X() * hitQ / 10.;
      Ym += pos.Y() * hitQ / 10.;
      Zm += pos.Z() * hitQ / 10.;
      Sxx += pos.X() * pos.X() * hitQ / 10.;
      Syy += pos.Y() * pos.Y() * hitQ / 10.;
      Szz += pos.Z() * pos.Z() * hitQ / 10.;
      Sxy += pos.X() * pos.Y() * hitQ / 10.;
      Sxz += pos.X() * pos.Z() * hitQ / 10.;
      Syz += pos.Y() * pos.Z() * hitQ / 10.;
   }

   Xm /= Q;
   Ym /= Q;
   Zm /= Q;
   Sxx /= Q;
   Syy /= Q;
   Szz /= Q;
   Sxy /= Q;
   Sxz /= Q;
   Syz /= Q;
   Sxx -= (Xm * Xm);
   Syy -= (Ym * Ym);
   Szz -= (Zm * Zm);
   Sxy -= (Xm * Ym);
   Sxz -= (Xm * Zm);
   Syz -= (Ym * Zm);

   theta = 0.5 * atan((2. * Sxy) / (Sxx - Syy));

   K11 = (Syy + Szz) * pow(cos(theta), 2) + (Sxx + Szz) * pow(sin(theta), 2) - 2. * Sxy * cos(theta) * sin(theta);
   K22 = (Syy + Szz) * pow(sin(theta), 2) + (Sxx + Szz) * pow(cos(theta), 2) + 2. * Sxy * cos(theta) * sin(theta);
   // K12 = -Sxy * (pow(cos(theta), 2) - pow(sin(theta), 2)) + (Sxx - Syy) * cos(theta) * sin(theta);
   K10 = Sxz * cos(theta) + Syz * sin(theta);
   K01 = -Sxz * sin(theta) + Syz * cos(theta);
   K00 = Sxx + Syy;

   c2 = -K00 - K11 - K22;
   c1 = K00 * K11 + K00 * K22 + K11 * K22 - K01 * K01 - K10 * K10;
   c0 = K01 * K01 * K11 + K10 * K10 * K22 - K00 * K11 * K22;

   p = c1 - pow(c2, 2) / 3.;
   q = 2. * pow(c2, 3) / 27. - c1 * c2 / 3. + c0;
   r = pow(q / 2., 2) + pow(p, 3) / 27.;

   if (r > 0)
      dm2 = -c2 / 3. + pow(-q / 2. + sqrt(r), 1. / 3.) + pow(-q / 2. - sqrt(r), 1. / 3.);
   else {
      rho = sqrt(-pow(p, 3) / 27.);
      phi = acos(-q / (2. * rho));
      dm2 = std::min(-c2 / 3. + 2. * pow(rho, 1. / 3.) * cos(phi / 3.),
                     std::min(-c2 / 3. + 2. * pow(rho, 1. / 3.) * cos((phi + 2. * TMath::Pi()) / 3.),
                              -c2 / 3. + 2. * pow(rho, 1. / 3.) * cos((phi + 4. * TMath::Pi()) / 3.)));
   }

   a = -K10 * cos(theta) / (K11 - dm2) + K01 * sin(theta) / (K22 - dm2);
   b = -K10 * sin(theta) / (K11 - dm2) - K01 * cos(theta) / (K22 - dm2);

   Xh = ((1. + b * b) * Xm - a * b * Ym + a * Zm) / (1. + a * a + b * b);
   Yh = ((1. + a * a) * Ym - a * b * Xm + b * Zm) / (1. + a * a + b * b);
   Zh = ((a * a + b * b) * Zm + a * Xm + b * Ym) / (1. + a * a + b * b);

   // First 3 are point1. Second 3 are point 2
   XYZPoint p1 = {Xm, Ym, Zm};
   XYZPoint p2 = {Xh, Yh, Zh};
   DefinePattern({p1, p2});
   fChi2 = (fabs(dm2 / Q));
   fNFree = points.size() - 6;
}
