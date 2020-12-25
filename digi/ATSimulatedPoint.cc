#include "ATSimulatedPoint.hh"

ClassImp(ATSimulatedPoint)

ATSimulatedPoint::ATSimulatedPoint()
{
  SetPoint(-1,-1, -1, -1, -1);
  
}

ATSimulatedPoint::ATSimulatedPoint(std::size_t id,Int_t electronNumber, Double_t x, Double_t y, Double_t atime)
{
  SetPoint(id,electronNumber, x, y, atime);
  fMCEventID=-1;//Undefined by construction
}

ATSimulatedPoint::~ATSimulatedPoint()
{}

void ATSimulatedPoint::SetElectronNumber(Int_t electronNumber)
{ 
  fElectronNumber = electronNumber; 
}

void ATSimulatedPoint::SetPosition(Double_t x, Double_t y, Double_t atime)
{ 
  fPosition = TVector3(x, y, atime);
}

void ATSimulatedPoint::SetPoint(std::size_t id,Int_t electronNumber, Double_t x, Double_t y, Double_t atime)
{ 
  fElectronNumber = electronNumber; fPosition = TVector3(x, y, atime); fPointID = id;
}

void ATSimulatedPoint::SetPointID(std::size_t id)
{ 
  fPointID = id;
}

void ATSimulatedPoint::SetMCEventID(std::size_t mcid)
{
  fMCEventID=mcid;
}

Int_t ATSimulatedPoint::GetElectronNumber()
{ 
  return fElectronNumber; 
}
TVector3 ATSimulatedPoint::GetPosition()
{ 
  return fPosition; 
}

std::size_t ATSimulatedPoint::GetPointID()
{ 
  return fPointID; 
}
std::size_t ATSimulatedPoint::GetMCEventID()
{
  return fMCEventID;
}
