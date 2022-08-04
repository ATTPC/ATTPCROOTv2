#include "AtPattern.h"

#include "AtContainerManip.h"
#include "AtHit.h" // for AtHit

#include <Math/Point3D.h> // for PositionVector3D
#include <TEveLine.h>
using namespace AtPatterns;

ClassImp(AtPattern);

AtPattern::AtPattern(Int_t numPoints) : fNumPoints(numPoints) {}

/**
 * @brief Fit the pattern.
 *
 * Fit the pattern shape using all hits in pointsToFit. Does a charge weighted fit if qThreshold is not equal to -1.
 * Sets fPatternPar parameters, and fChi2.
 *
 * @param[in] points Points in 3D space to fit with charge information
 * @param[in] qThreshold Only fit points that are above this charge threshold.
 * @return Chi-squared of the fit
 */
Double_t AtPattern::FitPattern(const std::vector<AtHit> &pointsToFit, Double_t qThreshold)
{
   FitPattern(ContainerManip::GetConstPointerVector(pointsToFit), qThreshold);
   return fChi2;
}

Double_t AtPattern::FitPattern(const std::vector<const AtHit *> &pointsToFit, Double_t qThreshold)
{
   std::vector<XYZPoint> points;
   std::vector<double> charge;
   for (auto hit : pointsToFit) {
      if (hit->GetCharge() > qThreshold) {
         points.push_back(hit->GetPosition());
         charge.push_back(hit->GetCharge());
      }
   }

   if (qThreshold == -1)
      FitPattern(points);
   else
      FitPattern(points, charge);
   return fChi2;
}
/**
 * @brief Fit the pattern shape.
 *
 * Fit the pattern shape using all points in pointsToFit. Does not weight for charge.
 * Sets fPatternPar parameters, and fChi2.
 *
 * @param[in] pointsToFit Points in 3D space to fit
 * @return Chi-squared of the fit
 */
Double_t AtPattern::FitPattern(const std::vector<XYZPoint> &pointsToFit)
{
   std::vector<double> charge;
   FitPattern(pointsToFit, charge);
   return fChi2;
}

TEveLine *AtPattern::GetEveLine(double tMin, double tMax, int n) const
{
   // Setup return vector with the correct number of
   auto retLine = new TEveLine(); // NOLINT these will be owned by TEve classes
   for (int i = 0; i < n; ++i) {
      auto t = tMin + i * (tMax - tMin) / n;
      auto pos = GetPointAt(t) / 10.; // TEve is all in units cm
      retLine->SetNextPoint(pos.X(), pos.Y(), pos.Z());
   }
   return retLine;
}
