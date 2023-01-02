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
   Int_t AdjecentPads = 1;  // Number of adjecent pads to be considered
   Int_t Items =2*AdjecentPads+1;
   Double_t R = 10.0e6; //Resistance (ohms)
   Double_t C =2.8e-7; // Capacitance[C] (Farad)
   Double_t t_amp = 1.32e-3; // time amplitude (microseconds)
   Double_t Dc = 0.8673; // Diffusion coefficent  (millimeter^2/microseconds)
   Double_t W = 2.2; // widith (millimeter)
   Float_t SigmaPercent = 1.0; // Sigma percent for charge dispersion
   

protected:
    Double_t ChargeDispersion( Double_t G, Double_t time ,Double_t x0,Double_t y0,Double_t xi,Double_t yi);
    virtual bool gatherElectronsFromSimulatedPoint(AtSimulatedPoint *line) override;

public:
   AtPulseTaskGADGET();
   AtPulseTaskGADGET(const char *name);
   ~AtPulseTaskGADGET()= default;
   void SetSigmaPercent(Float_t sigma){SigmaPercent = sigma;}
   virtual void Exec(Option_t *opt) override; //!< Executed for each event.

   ClassDefOverride(AtPulseTaskGADGET, 2);
};

#endif //#ifndef AtPULSETASKGADGET_H
