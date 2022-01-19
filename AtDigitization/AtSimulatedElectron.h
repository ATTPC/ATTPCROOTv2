#ifndef ATSIMULATEDELECTRON_H
#define ATSIMULATEDELECTRON_H

#include "AtSimulatedPoint.h"

class AtSimulatedElectron : public AtSimulatedPoint {
public:
   AtSimulatedElectron();
   AtSimulatedElectron(std::size_t id, Int_t electronNumber, Double_t x, Double_t y, Double_t atime);

   void SetPosition(Double_t x, Double_t y, Double_t atime);
   
   ClassDef(AtSimulatedElectron, 1);
};

#endif // ATSIMULATEDELECTRON_H
