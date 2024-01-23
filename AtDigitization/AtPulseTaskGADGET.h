/*****************************************************************/
/*    AtPulseTask: Simulates the ionized electrons which are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtPULSETASKGADGET_H
#define AtPULSETASKGADGET_H

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

class AtPulseTaskGADGET : public AtPulseTask {

private:
   Double_t G = 221;         // Gain[G] (V/Coulomb) should be rechecked 37-40
   Double_t R = 2.0e6;       // Resitsnce[R] (ohms)
   Double_t C = 2.8e-7;      // Capacitance[C] (Farad)
   Double_t t_amp = 1.32e-3; // time amplitude (microseconds)
   Double_t time = 45e-3;    // time[t_amp,t]
   Double_t Dc = 0.8673e-6;  // Diffusion coefficent [Dc] (meter^2/microseconds)
   Double_t W = 2.2e-3;      // widith [W] (meter)
   Int_t AdjecentPads = ;    // Number of adjecent pads to be considered`
   Double_t ChargeDispersion(Double_t x0, Double_t y0, Double_t xi, Double_t yi);
   virtual bool gatherElectronsFromSimulatedPoint(AtSimulatedPoint *line) override;

public:
   AtPulseTaskGADGET();
   AtPulseTaskGADGET(const char *name);
   ~AtPulseTaskGADGET() = default;

   ClassDefOverride(AtPulseTaskGADGET, 2);
};

#endif //#ifndef AtPULSETASKGADGET_H
