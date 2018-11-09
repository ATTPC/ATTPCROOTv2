#include "ATSimulatedPoint.hh"

ClassImp(ATSimulatedPoint)

ATSimulatedPoint::ATSimulatedPoint()
{
  SetPoint(-1, -1, -1, -1);
  
}

ATSimulatedPoint::ATSimulatedPoint(Int_t electronNumber, Double_t x, Double_t y, Double_t atime)
{
  SetPoint(electronNumber, x, y, atime);
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

void ATSimulatedPoint::SetPoint(Int_t electronNumber, Double_t x, Double_t y, Double_t atime)
{ 
  fElectronNumber = electronNumber; fPosition = TVector3(x, y, atime); 
}

Int_t ATSimulatedPoint::GetElectronNumber()
{ 
  return fElectronNumber; 
}
TVector3 ATSimulatedPoint::GetPosition()
{ 
  return fPosition; 
}
