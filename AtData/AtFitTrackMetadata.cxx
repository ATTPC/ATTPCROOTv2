#include "AtFitTrackMetadata.h"

#include <iostream>
ClassImp(AtFitTrackMetadata);

std::unique_ptr<AtFitTrackMetadata> AtFitTrackMetadata::Clone()
{
   return std::make_unique<AtFitTrackMetadata>(*this);
}

void AtFitTrackMetadata::Print() const

{
   std::cout << " Fit metadata " << fFitID << " for track with ID " << fTrackID << ":" << std::endl;

   std::cout << " Statistics: " << std::endl;
   std::cout << "   PValue    = " << fPValue << std::endl;
   std::cout << "   Chi2      = " << fChi2 << std::endl;
   std::cout << "   NDF       = " << fNdf << std::endl;
   std::cout << "   Converged = " << fFitConverged << std::endl;
}
