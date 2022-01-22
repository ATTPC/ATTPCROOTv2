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
   Int_t fCharge;

   ROOT::Math::XYZVector fPosition; //(mm,mm,us)

public:
   AtSimulatedPoint();
   AtSimulatedPoint(std::size_t mcPointID, Int_t clusterID, const ROOT::Math::XYZVector &pointLocation);
   AtSimulatedPoint(std::size_t mcPointID, Int_t clusterID, Int_t charge, const ROOT::Math::XYZVector &pointLocation);

   ~AtSimulatedPoint();

   void SetClusterID(Int_t clusterID);
   void SetMCPointID(std::size_t id);
   void SetMCEventID(std::size_t mcid);
   void SetCharge(Int_t charge);

   Int_t GetCharge();
   Int_t GetClusterID(); // Unique to each MCPoint in each MCEvent
   virtual ROOT::Math::XYZVector GetPosition();
   std::size_t GetMCPointID();
   std::size_t GetMCEventID();

   ClassDef(AtSimulatedPoint, 3);
};

#endif
