#ifndef AtPULSELINETASK_H
#define AtPULSELINETASK_H

#include "AtPulseTask.h"

#include <Rtypes.h>

#include <map>
#include <vector>

#include "Math/Vector3Dfwd.h"

class AtSimulatedLine;
class AtSimulatedPoint;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPulseLine : public AtPulse {

private:
   UInt_t fNumIntegrationPoints = 1000;
   UShort_t fNumSigmaToIntegrateZ = 3;

   std::map<Int_t, Float_t> fXYintegrationMap; //! xyIntegrationMap[padNum] = % of e- in event here

public:
   AtPulseLine(AtMapPtr map, ResponseFunc response = nullptr);

   void SetNumIntegrationPoints(UInt_t numPoints) { fNumIntegrationPoints = numPoints; }
   void SetNumSigmaToIntegrateZ(UShort_t zScore) { fNumSigmaToIntegrateZ = zScore; }
   UInt_t GetNumIntegrationPoints() { return fNumIntegrationPoints; }
   UShort_t SetNumSigmaToIntegrateZ() { return fNumSigmaToIntegrateZ; }

protected:
   void generateIntegrationMap(AtSimulatedLine &line);
   Int_t throwRandomAndGetPadAfterDiffusion(const ROOT::Math::XYZVector &loc, Double_t diffusionSigma);

   // Returns the bin ID (binMin) that the zIntegral starts from
   // fills zIntegral with the integral for bins starting with binMin, inclusive
   Int_t integrateTimebuckets(std::vector<double> &zIntegral, AtSimulatedLine *line);
   virtual bool AssignElectronsToPad(AtSimulatedPoint *line) override;
};
#endif
