#ifndef AtSIMULAtEDPOINT_H
#define AtSIMULAtEDPOINT_H

#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <Rtypes.h>
#include <TObject.h>

#include <cstddef>
#include <utility> // for swap

class TBuffer;
class TClass;
class TMemberInspector;

class AtSimulatedPoint : public TObject {

protected:
   //!< Cluster number
   std::size_t fMCPointID;
   std::size_t fMCEventID;
   Int_t fClusterID;
   Int_t fCharge{};

   ROOT::Math::XYZVector fPosition; //(mm,mm,us)

public:
   AtSimulatedPoint();
   AtSimulatedPoint(std::size_t mcPointID, Int_t clusterID, ROOT::Math::XYZVector pointLocation);
   AtSimulatedPoint(std::size_t mcPointID, Int_t clusterID, Int_t charge, ROOT::Math::XYZVector pointLocation);
   AtSimulatedPoint(const AtSimulatedPoint &object) = default;
   AtSimulatedPoint &operator=(AtSimulatedPoint other);

   friend void swap(AtSimulatedPoint &first, AtSimulatedPoint &second)
   {
      using std::swap;
      swap(first.fMCPointID, second.fMCPointID);
      swap(first.fMCEventID, second.fMCEventID);
      swap(first.fClusterID, second.fClusterID);
      swap(first.fCharge, second.fCharge);
      swap(first.fPosition, second.fPosition);
   }

   ~AtSimulatedPoint() = default;

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
