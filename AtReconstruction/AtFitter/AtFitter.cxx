#include "AtFitter.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

// STL
#include <algorithm>

ClassImp(AtFITTER::AtFitter)

   AtFITTER::AtFitter::AtFitter()
{
}

AtFITTER::AtFitter::~AtFitter() {}

std::tuple<Double_t, Double_t> AtFITTER::AtFitter::GetMomFromBrho(Double_t M, Double_t Z, Double_t brho)
{

   const Double_t M_Ener = M * 931.49401 / 1000.0;
   Double_t p = brho * Z * (2.99792458 / 10.0); // In GeV
   Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
   // std::cout << " Brho : " << brho << " - p : " << p << " - E : " << E << "\n";
   return std::make_tuple(p, E);
}

void AtFITTER::AtFitter::MergeTracks(std::vector<AtTrack> *trackCandSource, std::vector<AtTrack> *trackJunkSource,
                                     std::vector<AtTrack> *trackDest, bool fitDirection, bool simulationConv)
{

   // Track destination are the merged tracks.
   // Track candidate source are the main tracks identified as candidates.
   // Track junk source are the tracks from which clusters will be extracted. Boundary conditions are applied: Proximity
   // in space, angle and center.
   // NB: Only works for backward tracks

  
   Double_t trackDist = 20.0;  // Distance between clusters in mm
   Double_t angleSpread = 5.0; // Maximum angular spread between clusters
   Int_t minClusters = 3;
   Int_t trackSize = 0;

   for (auto trackCand : *trackCandSource) {
      Double_t thetaCand = trackCand.GetGeoTheta();
      auto hitArrayCand = trackCand.GetHitArray();
      std::pair<Double_t, Double_t> centerCand = trackCand.GetGeoCenter();

      AtTrack track = trackCand;
      Int_t jnkCnt = 0;
      Int_t jnkHitCnt = 0;

      if (simulationConv) {
         thetaCand = 180.0 - thetaCand * TMath::RadToDeg();

      } else {
         thetaCand = thetaCand * TMath::RadToDeg();
      }

      for (auto trackJunk : *trackJunkSource) {
         Double_t thetaJunk = trackJunk.GetGeoTheta();
         auto hitArrayJunk = trackJunk.GetHitArray();
         std::pair<Double_t, Double_t> centerJunk = trackJunk.GetGeoCenter();

         if (simulationConv) {

            thetaJunk = 180.0 - thetaJunk * TMath::RadToDeg();
         } else {

            thetaJunk = thetaJunk * TMath::RadToDeg();
         }

         if (fitDirection == 1 && thetaCand > 90) {

            if ((thetaCand + angleSpread) > thetaJunk && (thetaCand - angleSpread) < thetaJunk) {
               for (auto hit : *hitArrayJunk) {

                  track.AddHit(&hit);
                  ++jnkHitCnt;
               }
            }

         } else if (fitDirection == 0 && thetaCand < 90) {
         }

         ++jnkCnt;

      } // Junk track

      // track.SortClusterHitArrayZ();
      track.SortHitArrayTime();

      // Prune if other tracks were added track

      if (trackJunkSource->size() > 0) {
         Double_t pruneFraction = 10.0;
         if (jnkHitCnt > hitArrayCand->size())
            pruneFraction = 4.0; // 25%
         else
            pruneFraction = 10.0; // 10%

         Int_t numHits = (Int_t)track.GetHitArray()->size() / pruneFraction;
         for (auto iHit = 0; iHit < numHits; ++iHit)
            track.GetHitArray()->pop_back();
      }

      trackDest->push_back(track);

   } // Source track
}
