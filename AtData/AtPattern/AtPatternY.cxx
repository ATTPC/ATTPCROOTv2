#include "AtPatternY.h"

#include <FairLogger.h>

#include <Math/Vector3D.h> // for DisplacementVector3D, operator*

#include <cmath>   // for cos, sin, pow, sqrt, acos, atan, fabs
#include <utility> // for swap

class TEveLine;

using namespace AtPatterns;

ClassImp(AtPatternY);

AtPatternY::AtPatternY() : AtPattern(4) {}

TEveLine *AtPatternY::GetEveLine() const
{
   return nullptr;
}

AtPatternY::XYZVector AtPatternY::GetDirection(int line) const
{
   if (line < 3) {
      return {fPatternPar[3 + 3 * line], fPatternPar[4 + 3 * line], fPatternPar[5 + 3 * line]};
   } else {
      LOG(error) << "Line number, " << line << ", is greater than the number of actual lines, 3.";
      return {0, 0, 0};
   }
}

AtPatternY::XYZPoint AtPatternY::ClosestPointOnPattern(const XYZPoint &point) const
{
   auto p = GetVertex();

   return p;
}

Double_t AtPatternY::DistanceToPattern(const XYZPoint &point) const
{
   auto vec = GetVertex() - point;
   double minDist = 1000;
   for (int i = 0; i < 3; i++) {
      double dist2;
      if ((i == 0 && point.Z() > GetVertex().Z()) || (i > 0 && point.Z() < GetVertex().Z())) {
         auto nD = GetDirection(i).Cross(vec);
         dist2 = nD.Mag2() / GetDirection(i).Mag2();
      } else {
         dist2 = vec.Mag2();
      }
      if (dist2 < minDist) {
         minDist = dist2;
      }
   }

   return std::sqrt(minDist);
}

void AtPatternY::DefinePattern(const std::vector<XYZPoint> &points)
{
   // std::cout << "making pattern" << std::endl;
   if (points.size() != fNumPoints)
      LOG(error) << "Trying to create model with wrong number of points " << points.size();

   int zPosIndx[4] = {};
   double zPos[4] = {};

   for (int i = 0; i < 4; i++) {
      zPosIndx[i] = i;
      zPos[i] = points[i].Z();
      // std::cout << "1 " << i << std::endl;
   }
   for (int i = 0; i < 3; i++) {
      // std::cout << "2 " << i << std::endl;
      for (int r = 0; r < 3; r++) {
         // std::cout << "2 " << i << std::endl;
         // std::cout << 4 - i << std::endl;
         // std::cout << "3 " << r << std::endl;
         if (zPos[r] > zPos[r + 1]) {
            std::swap(zPos[r], zPos[r + 1]);
            std::swap(zPosIndx[r], zPosIndx[r + 1]);
         }
      }
   }
   for (int i = 0; i < 4; i++) {
      // std::cout << zPos[i] << "  ";
   }
   // std::cout << std::endl;
   for (int i = 0; i < 4; i++) {
      // std::cout << zPosIndx[i] << "  ";
   }
   // std::cout << std::endl;
   auto fVertex = points[zPosIndx[2]];
   XYZPoint fDirection[3];
   int j = 0;
   for (int i = 3; i >= 0; i--) {
      // std::cout << "4 " << i << std::endl;
      if (i != 2) {
         fDirection[j] = points[zPosIndx[i]] - fVertex;
         // If not perpendicular to z-axis rescale direction
         if (fDirection[j].Z() != 0) {
            fDirection[j] /= fDirection[j].Z();
         }
         j++;
      }
   }

   fPatternPar = {fVertex.X(),       fVertex.Y(),       fVertex.Z(),       fDirection[0].X(),
                  fDirection[0].Y(), fDirection[0].Z(), fDirection[1].X(), fDirection[1].Y(),
                  fDirection[1].Z(), fDirection[2].X(), fDirection[2].Y(), fDirection[2].Z()};
}

/**
 * @brief Get point on line at z
 *
 * Get point on line at z. If the line is parallel to Z, then return then the parameter passed
 * has not defined physical interpretation
 *
 * @param[in] z Location of point at z [mm]
 */
AtPatternY::XYZPoint AtPatternY::GetPointAt(double z) const
{
   return GetVertex() + z * GetDirection(0);
}

void AtPatternY::FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge)
{
   //------3D Line Regression
   //----- adapted from: http://fr.scribd.com/doc/31477970/Regressions-et-trajectoires-3D
   double Q = 0.;
   double dm2 = 0.;

   double total_charge = 0;
   bool doChargeWeight = points.size() == charge.size();

   for (int i = 0; i < points.size(); ++i) {
      const auto hitQ = doChargeWeight ? charge[i] : 1;
      Q += hitQ / 10.;
      dm2 += pow(DistanceToPattern(points[i]), 2) / hitQ / 10.;
   }

   fChi2 = (fabs(dm2 / Q));
   fNFree = points.size() - 6;
}
