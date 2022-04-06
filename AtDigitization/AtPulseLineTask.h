/*****************************************************************/
/*    AtPulseTask: Simulates the ionized electrons which are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtPULSELINETASK_H
#define AtPULSELINETASK_H

#include <Rtypes.h>
#include <map>
#include <vector>

#include "AtPulseTask.h"
#include "Math/Vector3Dfwd.h"

class AtSimulatedLine;
class AtSimulatedPoint;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPulseLineTask : public AtPulseTask {

private:
   UInt_t fNumIntegrationPoints = 1000;
   UShort_t fNumSigmaToIntegrateZ = 3;

   std::map<Int_t, Float_t> fXYintegrationMap; //! xyIntegrationMap[padNum] = % of e- in event here

   void generateIntegrationMap(AtSimulatedLine *line);
   Int_t throwRandomAndGetBinAfterDiffusion(const ROOT::Math::XYZVector &loc, Double_t diffusionSigma);

   // Returns the bin ID (binMin) that the zIntegral starts from
   // fills zIntegral with the integral for bins starting with binMin, inclusive
   Int_t integrateTimebuckets(std::vector<double> &zIntegral, AtSimulatedLine *line);
   virtual bool gatherElectronsFromSimulatedPoint(AtSimulatedPoint *line) override;

public:
   AtPulseLineTask();
   ~AtPulseLineTask();

   void SetNumIntegrationPoints(UInt_t numPoints) { fNumIntegrationPoints = numPoints; }
   void SetNumSigmaToIntegrateZ(UShort_t zScore) { fNumSigmaToIntegrateZ = zScore; }
   UInt_t GetNumIntegrationPoints() { return fNumIntegrationPoints; }
   UShort_t SetNumSigmaToIntegrateZ() { return fNumSigmaToIntegrateZ; }

   ClassDefOverride(AtPulseLineTask, 2);
};

#endif //#ifndef AtPULSELINETASK_H
