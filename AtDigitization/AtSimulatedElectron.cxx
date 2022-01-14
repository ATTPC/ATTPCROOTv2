#include "AtSimulatedElectron.h"

AtSimulatedElectron::AtSimulatedElectron() : AtSimulatedPoint()
{
   SetCharge(1);
}

AtSimulatedElectron::AtSimulatedElectron(std::size_t id, Int_t electronNumber, Double_t x, Double_t y, Double_t atime)
   : AtSimulatedPoint(id, electronNumber, 1, x, y, atime, 0)
{
}

ClassImp(AtSimulatedElectron)
