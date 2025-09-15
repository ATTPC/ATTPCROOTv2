#include "AtBraggFitMetadata.h"

#include <iostream>
ClassImp(AtBraggFitMetadata);

void AtBraggFitMetadata::Print() const
{
   AtFitTrackMetadata::Print();

   std::cout << " Bragg fitting specifics: " << std::endl;
   std::cout << "   ELoss model name: " << fELossModelName.Data() << std::endl;
   std::cout << "   Particle: (A, Z) = (" << fA << ", " << fZ << "), mass = " << fMassAmu << " umas" << std::endl;
   std::cout << "   KineticEnergy = (" << fKineticEnergy << " +- " << fKineticEnergyUncertainty << ") MeV" << std::endl;
   std::cout << "   AmplitudeFactor = (" << fAmplitudeFactor << " +- " << fAmplitudeFactorUncertainty << ") ADC/MeV" <<std::endl;
}
