#include "AtPPACPair.h"

ClassImp(AtPPACPair);

void AtPPACPair::CalculateDirection()
{
   // Entrance and exit positions relative to the center of the upstream PPAC.
   XYZPoint initialPoint(fEntrancePosition.X(), fEntrancePosition.Y(), 0);
   XYZPoint finalPoint(fExitPosition.X(), fExitPosition.Y(), fSeparationDistance);

   // Direction vector.
   fTrackDirection = finalPoint - initialPoint;

   // Set angles based off the direction vector.
   fTrackPolarAngle = fTrackDirection.Theta();
   fTrackAzimutalAngle = fTrackDirection.Phi();
}

std::unique_ptr<AtPPACPair> AtPPACPair::Clone()
{
   return std::make_unique<AtPPACPair>(*this);
}
