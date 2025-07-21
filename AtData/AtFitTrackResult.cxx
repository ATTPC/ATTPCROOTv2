#include "AtFitTrackResult.h"

#include <iostream>
ClassImp(AtFitTrackResult);

void AtFitTrackResult::Print() const

{
   std::cout << " Fit result for track with ID " << fTrackID << ":" << std::endl;

   std::cout << " Statistics: " << std::endl;
   std::cout << "   PValue    = " << fPValue << std::endl;
   std::cout << "   Chi2      = " << fChi2 << std::endl;
   std::cout << "   NDF       = " << fNdf << std::endl;
   std::cout << "   Converged = " << fFitConverged << std::endl;
}
