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

TEveLine *AtPattern::GetEveLine(Double_t deltaR, XYZPoint firstPt, XYZPoint lastPt) const
{
   // Setup return vector with the correct number of
   auto retLine = new TEveLine(); // NOLINT these will be owned by TEve classes
   Double_t tMin=0,tMax=0,deltaPar=0.01;

   deltaPar = deltaR/sqrt(pow(fPatternPar[3],2) + pow(fPatternPar[4],2) + pow(fPatternPar[5],2));
   tMin=(firstPt.X()-fPatternPar[0])/fPatternPar[3];
   tMax=(lastPt.X()-fPatternPar[0])/fPatternPar[3];
   tMin-=10; tMax+=10;
   std::vector<Double_t> tminmax;
   tminmax = lineIntersecR(250,tMin,tMax);//250 is the max. radius where the lines are stopped
   if(tminmax.size()==2) {
     tMin=tminmax.at(0);
     tMax=tminmax.at(1);
   }
   if(tMin>tMax){
     Double_t tBuff=tMax;
     tMax=tMin;
     tMin=tBuff;
   }
   for(int i=0; i<=(Int_t)((tMax-tMin)/deltaPar); i++) {
      auto t = tMin + i*deltaPar;
      auto pos = GetPointAt(t) / 10.; // TEve is all in units cm
      retLine->SetNextPoint(pos.X(), pos.Y(), pos.Z());
   }
   return retLine;
}

std::vector<Double_t> AtPattern::lineIntersecR(Double_t rMax, Double_t tMin, Double_t tMax) const
{
  std::vector<Double_t> result;
  Double_t a=0,b=0,c=0,sol1=0,sol2=0;
	a = pow(fPatternPar[3],2) + pow(fPatternPar[4],2);
	b = 2*(fPatternPar[0]*fPatternPar[3] + fPatternPar[1]*fPatternPar[4]);
	c = pow(fPatternPar[0],2) + pow(fPatternPar[1],2) - pow(rMax,2);
	if(b*b-4*a*c>=0){
		sol1=(-b+sqrt(b*b-4*a*c))/(2*a);
		sol2=(-b-sqrt(b*b-4*a*c))/(2*a);
    result.push_back(sol1);
    result.push_back(sol2);
	}
  return result;
}
