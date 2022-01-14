#include "AtSimulatedPoint.h"

using XYZVector = ROOT::Math::XYZVector;

ClassImp(AtSimulatedPoint);

AtSimulatedPoint::AtSimulatedPoint()
{
   SetPoint(-1, -1, -1, -1, -1, -1, -1);
}

AtSimulatedPoint::AtSimulatedPoint(std::size_t mcPointID, Int_t clusterID, Int_t charge, Double_t x, Double_t y,
                                   Double_t atime, Double_t longitudinalDiff)
{
   SetPoint(mcPointID, clusterID, charge, x, y, atime, longitudinalDiff);
}

AtSimulatedPoint::~AtSimulatedPoint() {}

void AtSimulatedPoint::SetClusterID(Int_t clusterID)
{
   fClusterID = clusterID;
}

void AtSimulatedPoint::SetPosition(Double_t x, Double_t y, Double_t atime)
{
   fPosition = XYZVector(x, y, atime);
}

void AtSimulatedPoint::SetPoint(std::size_t mcPointID, Int_t clusterID, Int_t charge, Double_t x, Double_t y,
                                Double_t atime, Double_t longitudinalDiffusionSigma)
{
   SetClusterID(clusterID);
   SetPosition(x, y, atime);
   SetMCEventID(-1); // Undefined by construction (AA does not know what the purpose is)
   SetMCPointID(mcPointID);
   SetCharge(charge);
}
void AtSimulatedPoint::SetCharge(Int_t charge)
{
   fCharge = charge;
}
void AtSimulatedPoint::SetMCPointID(std::size_t id)
{
   fMCPointID = id;
}
void AtSimulatedPoint::SetMCEventID(std::size_t mcid)
{
   fMCEventID = mcid;
}
void AtSimulatedPoint::SetLongitudinalDiffusion(Double_t sigma)
{
   fSigmaLongDiffusion = sigma;
}

Int_t AtSimulatedPoint::GetClusterID()
{
   return fClusterID;
}
XYZVector AtSimulatedPoint::GetPosition()
{
   return fPosition;
}
Int_t AtSimulatedPoint::GetCharge()
{
   return fCharge;
}
std::size_t AtSimulatedPoint::GetMCPointID()
{
   return fMCPointID;
}
std::size_t AtSimulatedPoint::GetMCEventID()
{
   return fMCEventID;
}
Double_t AtSimulatedPoint::GetLongitudinalDiffusion()
{
   return fSigmaLongDiffusion;
}
