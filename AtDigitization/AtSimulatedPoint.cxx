#include "AtSimulatedPoint.h"

ClassImp(AtSimulatedPoint)

   AtSimulatedPoint::AtSimulatedPoint()
{
   SetPoint(-1, -1, -1, -1, -1);
}

AtSimulatedPoint::AtSimulatedPoint(std::size_t id, Int_t electronNumber, Double_t x, Double_t y, Double_t atime)
{
   SetPoint(id, electronNumber, x, y, atime);
   fMCEventID = -1; // Undefined by construction
}

AtSimulatedPoint::~AtSimulatedPoint() {}

void AtSimulatedPoint::SetElectronNumber(Int_t electronNumber)
{
   fElectronNumber = electronNumber;
}

void AtSimulatedPoint::SetPosition(Double_t x, Double_t y, Double_t atime)
{
   fPosition = TVector3(x, y, atime);
}

void AtSimulatedPoint::SetPoint(std::size_t id, Int_t electronNumber, Double_t x, Double_t y, Double_t atime)
{
   fElectronNumber = electronNumber;
   fPosition = TVector3(x, y, atime);
   fPointID = id;
}

void AtSimulatedPoint::SetPointID(std::size_t id)
{
   fPointID = id;
}

void AtSimulatedPoint::SetMCEventID(std::size_t mcid)
{
   fMCEventID = mcid;
}

Int_t AtSimulatedPoint::GetElectronNumber()
{
   return fElectronNumber;
}
TVector3 AtSimulatedPoint::GetPosition()
{
   return fPosition;
}

std::size_t AtSimulatedPoint::GetPointID()
{
   return fPointID;
}
std::size_t AtSimulatedPoint::GetMCEventID()
{
   return fMCEventID;
}
