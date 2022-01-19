#ifndef AtSIMULATEDLINE_H
#define AtSIMULATEDLINE_H

#include "AtSimulatedPoint.h"
#include "Math/Vector3D.h"

class AtSimulatedLine : public AtSimulatedPoint {

protected:
   Double_t fSigmaLongDiffusion;
   Double_t fSigmaTransDiffusion;
   ROOT::Math::XYZVector fPositionFinal; //(mm,mm,us)

public:
   AtSimulatedLine();
   AtSimulatedLine(std::size_t mcPointID, Int_t clusterID, Int_t charge,
		   const ROOT::Math::XYZVector &posIn, const ROOT::Math::XYZVector &posOut,
		   Double_t longitudalDiffusionSigma, Double_t transverseDiffusionSigma);

   void SetFinalPosition(Double_t x, Double_t y, Double_t zTime);
   void SetInitialPosition(Double_t x, Double_t y, Double_t zTime);
   void SetTransverseDiffusion(Double_t sigma);
   void SetLongitudinalDiffusion(Double_t sigma);
   
   ROOT::Math::XYZVector GetFinalPosition();
   ROOT::Math::XYZVector GetInitialPosition();
   ROOT::Math::XYZVector GetPosition() override;
   Double_t GetTransverseDiffusion();
   Double_t GetLongitudinalDiffusion();

   ClassDefOverride(AtSimulatedLine, 1);
};

#endif //#ifndef AtSIMULATEDLINE_H
