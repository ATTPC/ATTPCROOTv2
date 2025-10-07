#ifndef ATPUNCHTHROUGHCHECKER_H
#define ATPUNCHTHROUGHCHECKER_H

#include "AtTrack.h"
#include "AtHit.h"

#include <tuple>

namespace AtTools {

/**
 * Class used to check if a given track has stopped inside the TPC volume or if it has punched though.
 * For now, this will be hardcoded to work only for ATTPC, although it may also work for other cylindrical TPCs.
 * The way this is checked is by looking at the position of the last hit in the AtTrack. It this last hit is too close
 * to the TPC border, given a distance threshold, we consider that it has punched through.
 */

class AtPunchThroughChecker {
protected:
   // Geometry of the TPC.
   double fTPCLength{1000}; // mm
   double fTPCRadius{250};  // mm

   // If the last hit of the AtTrack is closer than this distance to the border of the TPC, it is considered that is has punched through.
   double fDistanceThreshold{20}; // mm

public:
   AtPunchThroughChecker() = default;
   ~AtPunchThroughChecker() = default;

   bool IsPunchThrough(AtTrack *track);

   void SetTPCLength(double value) { fTPCLength = value; }
   void SetTPCRadius(double value) { fTPCRadius = value; }
   void SetDistanceThreshold(double value) { fDistanceThreshold = value; }

private:
   std::tuple<double, double> GetLastHitZPosAndRadius(AtTrack *track, bool &movingForward);

};
} // namespace AtTools

#endif
