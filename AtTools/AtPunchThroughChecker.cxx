#include "AtPunchThroughChecker.h"

#include <TMath.h>

bool AtTools::AtPunchThroughChecker::IsPunchThrough(AtTrack *track)
{
   // First guess is that it does not punch through.
   bool punchThrough{false};

   // Get the last hit and check if the track is going forward or backwards.
   bool movingForward{true};
   auto [lastHitZPos, lastHitRadius] = GetLastHitZPosAndRadius(track, movingForward);

   // Too close to border according to radius.
   if (lastHitRadius > fTPCRadius - fDistanceThreshold)
      return true;

   // Check which plane is the AtTrack moving towards to.
   double borderPlaneZPos{fTPCLength};
   if (movingForward)
      borderPlaneZPos = 0;

   // Too close to border according to Z position.
   double distanceToBorderPlane = TMath::Abs(borderPlaneZPos - lastHitZPos);
   if (distanceToBorderPlane < fDistanceThreshold)
      return true;

   // If every if statement failed, then it did not punch through.
   return false;
}

std::tuple<double, double> AtTools::AtPunchThroughChecker::GetLastHitZPosAndRadius(AtTrack *track, bool &movingForward)
{
   // Extraxt the AtHit vector.
   const auto &hits = track->GetHitArray();

   // Find the index of the AtHits with minimum and maximum radius.
   double minRadiusSquared{fTPCRadius * fTPCRadius * 1.1};
   double maxRadiusSquared{0};
   double minRadiusZPosition{-1};
   double maxRadiusZPosition{-1};

   int idxMaxRadius{0};
   int idx{0};
   for (auto &hit : hits) {
      auto currentPosition = hit->GetPosition();
      double currentRadiusSquared = currentPosition.Perp2();

      if (currentRadiusSquared < minRadiusSquared) {
         minRadiusSquared = currentRadiusSquared;
         minRadiusZPosition = currentPosition.Z();
      }

      if (currentRadiusSquared > maxRadiusSquared) {
         maxRadiusSquared = currentRadiusSquared;
         maxRadiusZPosition = currentPosition.Z();
         idxMaxRadius = idx;
      }

      idx++;
   }

   // Beam enters from Z=1000mm. Check if the track goes forward or not.
   if (minRadiusZPosition < maxRadiusZPosition)
      movingForward = false;

   // Return the (Z, radius) tuple of the last AtHit.
   return std::make_tuple(maxRadiusZPosition, TMath::Sqrt(maxRadiusSquared));
}
