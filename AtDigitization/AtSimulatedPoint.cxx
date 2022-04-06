#include "AtSimulatedPoint.h"

#include "Rtypes.h"

using XYZVector = ROOT::Math::XYZVector;

ClassImp(AtSimulatedPoint);

AtSimulatedPoint::AtSimulatedPoint() : fMCEventID(-1), fMCPointID(-1), fClusterID(-1) {}

AtSimulatedPoint::AtSimulatedPoint(std::size_t mcPointID, Int_t clusterID, const XYZVector &position)
   : fPosition(position), fMCEventID(-1), fMCPointID(mcPointID), fClusterID(clusterID), fCharge(1)
{
}

AtSimulatedPoint::AtSimulatedPoint(std::size_t mcPointID, Int_t clusterID, Int_t charge, const XYZVector &position)
   : fPosition(position), fMCEventID(-1), fMCPointID(mcPointID), fClusterID(clusterID), fCharge(charge)
{
}

AtSimulatedPoint::~AtSimulatedPoint() {}

void AtSimulatedPoint::SetClusterID(Int_t clusterID)
{
   fClusterID = clusterID;
}
void AtSimulatedPoint::SetMCPointID(std::size_t id)
{
   fMCPointID = id;
}
void AtSimulatedPoint::SetMCEventID(std::size_t mcid)
{
   fMCEventID = mcid;
}

Int_t AtSimulatedPoint::GetClusterID()
{
   return fClusterID;
}
XYZVector AtSimulatedPoint::GetPosition()
{
   return fPosition;
}
std::size_t AtSimulatedPoint::GetMCPointID()
{
   return fMCPointID;
}
std::size_t AtSimulatedPoint::GetMCEventID()
{
   return fMCEventID;
}
void AtSimulatedPoint::SetCharge(Int_t charge)
{
   fCharge = charge;
}
Int_t AtSimulatedPoint::GetCharge()
{
   return fCharge;
}
