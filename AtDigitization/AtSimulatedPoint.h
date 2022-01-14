#ifndef AtSIMULAtEDPOINT_H
#define AtSIMULAtEDPOINT_H

//#include "TROOT.h"
#include "TObject.h"
#include "Math/Vector3D.h"

class AtSimulatedPoint : public TObject {

protected:
   //!< Cluster number
   std::size_t fMCPointID;
   std::size_t fMCEventID;
   Int_t fClusterID;
   Int_t fCharge; // in units e-
   Double_t fSigmaLongDiffusion;

   ROOT::Math::XYZVector fPosition;

public:
   AtSimulatedPoint();
   AtSimulatedPoint(std::size_t mcPointID, Int_t clusterID, Int_t charge, Double_t x, Double_t y, Double_t atime,
                    Double_t longitudinalDiffusionSigma);
   ~AtSimulatedPoint();

   void SetClusterID(Int_t clusterID);
   void SetPoint(std::size_t mcPointID, Int_t clusterID, Int_t charge, Double_t x, Double_t y, Double_t atime,
                 Double_t longitudinalDiffusionSigma);
   void SetPosition(Double_t x, Double_t y, Double_t atime);
   void SetCharge(Int_t charge);
   void SetMCPointID(std::size_t id);
   void SetMCEventID(std::size_t mcid);
   void SetLongitudinalDiffusion(Double_t sigma);

   Int_t GetClusterID(); // Unique to each MCPoint in each MCEvent
   ROOT::Math::XYZVector GetPosition();
   Int_t GetCharge();
   Double_t GetLongitudinalDiffusion();
   std::size_t GetMCPointID();
   std::size_t GetMCEventID();

   ClassDef(AtSimulatedPoint, 2);
};

#endif
