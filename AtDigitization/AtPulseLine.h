#ifndef AtPULSELINETASK_H
#define AtPULSELINETASK_H

#include "AtPulse.h" // for AtPulse::ResponseFunc, AtPulse, AtPuls...

#include <map>
#include <memory> // for make_shared, shared_ptr
#include <sys/types.h>
#include <tuple>   // for tie
#include <utility> // for pair
#include <vector>

#include "Math/Vector3Dfwd.h"

class AtSimulatedLine;
class AtSimulatedPoint;

class AtPulseLine : public AtPulse {

private:
   uint fNumIntegrationPoints = 1000; ///< Number of points to sample the transverse diffusion for each line
   ushort fNumSigmaToIntegrateZ = 3;  ///< Number of sigma to integrate the longitudinal diffusion for each line
   bool fUseFixedSample = false;
   std::vector<double> fPatternX; // Unit pattern of the fixed diffusion sampling (only filled if it is enabled)
   std::vector<double> fPatternY;

   std::map<int, float> fXYintegrationMap; //! xyIntegrationMap[padNum] = % of e- in event here

public:
   AtPulseLine(AtMapPtr map, ResponseFunc response = nullptr);
   AtPulseLine(const AtPulseLine &other) = default;

   void SetNumIntegrationPoints(uint numPoints)
   {
      fNumIntegrationPoints = numPoints;
      if (fUseFixedSample)
         std::tie(fPatternX, fPatternY) = MakeUnitPattern(fNumIntegrationPoints);
   }
   /**
    * Sample the transverse diffusion with a fixed pattern of fNumIntegrationPoints points (distributed as the random
    * points: radius |Gaus(0, sigma)|, uniform angle) instead of independent random points.
    *
    * This allows for sampling the average effect of transverse diffusion to the same accuracy with fewer points than
    * random sampling, and without the need for RNG calls (but does not capture fluctuations between MCPoints).
    * The pattern gets one random rotation per MCPoint so its orientation is not fixed against the pad edges.
    */
   void SetUseFixedSample(bool val)
   {
      fUseFixedSample = val;
      if (val)
         std::tie(fPatternX, fPatternY) = MakeUnitPattern(fNumIntegrationPoints);
   }

   void SetNumSigmaToIntegrateZ(ushort zScore) { fNumSigmaToIntegrateZ = zScore; }
   uint GetNumIntegrationPoints() { return fNumIntegrationPoints; }
   ushort SetNumSigmaToIntegrateZ() { return fNumSigmaToIntegrateZ; }
   virtual std::shared_ptr<AtPulse> Clone() const override { return std::make_shared<AtPulseLine>(*this); }

protected:
   void generateIntegrationMap(AtSimulatedLine &line);
   int throwRandomAndGetPadAfterDiffusion(const ROOT::Math::XYZVector &loc, double diffusionSigma);

   /**
    * Generates a pattern of 2D points around the origin approximating a random draw. The radii are
    * deterministically spaced to follow a |N(0,1)| distribution, and each point is rotated
    * from the previous one by the golden angle so the points are evenly distributed in angle.
    */
   static std::pair<std::vector<double>, std::vector<double>> MakeUnitPattern(int n);

   // Returns the bin ID (binMin) that the zIntegral starts from
   // fills zIntegral with the integral for bins starting with binMin, inclusive
   int integrateTimebuckets(std::vector<double> &zIntegral, AtSimulatedLine *line);
   virtual bool AssignElectronsToPad(AtSimulatedPoint *line) override;
};
#endif
