#ifndef AtPULSELINETASK_H
#define AtPULSELINETASK_H

#include "AtPulse.h" // for AtPulse::ResponseFunc, AtPulse, AtPuls...

#include <map>
#include <memory> // for make_shared, shared_ptr
#include <sys/types.h>
#include <vector>

#include "Math/Vector3Dfwd.h"

class AtSimulatedLine;
class AtSimulatedPoint;

class AtPulseLine : public AtPulse {

private:
   uint fNumIntegrationPoints = 1000;
   ushort fNumSigmaToIntegrateZ = 3;

   std::map<int, float> fXYintegrationMap; //! xyIntegrationMap[padNum] = % of e- in event here

public:
   AtPulseLine(AtMapPtr map, ResponseFunc response = nullptr);
   AtPulseLine(const AtPulseLine &other) = default;

   void SetNumIntegrationPoints(uint numPoints) { fNumIntegrationPoints = numPoints; }
   void SetNumSigmaToIntegrateZ(ushort zScore) { fNumSigmaToIntegrateZ = zScore; }
   uint GetNumIntegrationPoints() { return fNumIntegrationPoints; }
   ushort SetNumSigmaToIntegrateZ() { return fNumSigmaToIntegrateZ; }
   virtual std::shared_ptr<AtPulse> Clone() const override { return std::make_shared<AtPulseLine>(*this); }

protected:
   void generateIntegrationMap(AtSimulatedLine &line);
   int throwRandomAndGetPadAfterDiffusion(const ROOT::Math::XYZVector &loc, double diffusionSigma);

   // Returns the bin ID (binMin) that the zIntegral starts from
   // fills zIntegral with the integral for bins starting with binMin, inclusive
   int integrateTimebuckets(std::vector<double> &zIntegral, AtSimulatedLine *line);
   virtual bool AssignElectronsToPad(AtSimulatedPoint *line) override;
};
#endif
