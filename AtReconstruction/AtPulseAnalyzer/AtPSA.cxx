#include "AtPSA.h"

#include <FairLogger.h>

#include <algorithm>
#include <iostream>
#include <iterator>

// FairRoot classes
#include <FairRun.h>
#include <FairRuntimeDb.h>
// ROOT classes
#include "AtCalibration.h"
#include "AtDigiPar.h"
#include "AtHit.h"
#include "AtMCPoint.h"

#include <Rtypes.h>
#include <TClonesArray.h>
#include <TMath.h>
#include <TObject.h> // for TObject
#include <TVector3.h>

using std::distance;
using std::max_element;
using std::min_element;

void AtPSA::Init()
{

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(FATAL) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb(); // NOLINT
   if (!db)
      LOG(FATAL) << "No runtime database!";

   auto fPar = (AtDigiPar *)db->getContainer("AtDigiPar"); // NOLINT
   if (!fPar)
      LOG(FATAL) << "AtDigiPar not found!!";

   fTBTime = fPar->GetTBTime();
   fDriftVelocity = fPar->GetDriftVelocity();
   fBField = fPar->GetBField();
   fEField = fPar->GetEField();
   fZk = fPar->GetZPadPlane();
   fEntTB = (Int_t)fPar->GetTBEntrance();

   auto timeToPadPlane = fZk / (fDriftVelocity * 1e-2); //[ns]
   fTB0 = fEntTB - timeToPadPlane / fTBTime;

   std::cout << " ==== Parameters for Pulse Shape Analysis Task ==== " << std::endl;
   std::cout << " ==== Magnetic Field : " << fBField << " T " << std::endl;
   std::cout << " ==== Electric Field : " << fEField << " V/cm " << std::endl;
   std::cout << " ==== Sampling Rate : " << fTBTime << " ns " << std::endl;
   std::cout << " ==== Drift Velocity : " << fDriftVelocity << " cm/us " << std::endl;
   std::cout << " ==== Entrance TB : " << fEntTB << std::endl;
   std::cout << " ==== Pad plane TB : " << fTB0 << std::endl;
   std::cout << " ==== NumTbs : " << fNumTbs << std::endl;
}

void AtPSA::SetSimulatedEvent(TClonesArray *MCSimPointArray)
{
   fMCSimPointArray = MCSimPointArray;
}

void AtPSA::SetThreshold(Int_t threshold)
{
   fThreshold = threshold;
   if (!fUsingLowThreshold)
      fThresholdlow = threshold;
}

void AtPSA::SetThresholdLow(Int_t thresholdlow)
{
   fThresholdlow = thresholdlow;
   fUsingLowThreshold = kTRUE;
}

Double_t AtPSA::CalculateZ(Double_t peakIdx)
{
   // DEPRECAtED
   return (fNumTbs - peakIdx) * fTBTime * fDriftVelocity / 100.;
}

Double_t AtPSA::CalculateZGeo(Double_t peakIdx)
{

   // This function must be consistent with the re-calibrations done before.
   return fZk - (fEntTB - peakIdx) * fTBTime * fDriftVelocity / 100.;
}

void AtPSA::TrackMCPoints(std::multimap<Int_t, std::size_t> &map, AtHit &hit)
{
   auto padNum = hit.GetPadNum();
   for (auto it = map.lower_bound(padNum); it != map.upper_bound(padNum); ++it) {

      if (fMCSimPointArray != nullptr) {
         auto *MCPoint = dynamic_cast<AtMCPoint *>(fMCSimPointArray->At(it->second));

         AtHit::MCSimPoint mcpoint(it->second, MCPoint->GetTrackID(), MCPoint->GetEIni(), MCPoint->GetEnergyLoss(),
                                   MCPoint->GetAIni(), MCPoint->GetMassNum(), MCPoint->GetAtomicNum());
         hit.AddMCSimPoint(mcpoint);

         // std::cout << " Pad Num : "<<hit->GetHitPadNum()<<" MC Point ID : "<<it->second << std::endl;
         // std::cout << " Track ID : "<<MCPoint->GetTrackID()<<" Energy (MeV) : "<<MCPoint->GetEIni()<<" Angle (deg) :
         // "<<MCPoint->GetAIni()<<"\n"; std::cout << " Mass Number : "<<MCPoint->GetMassNum()<<" Atomic Number
         // "<<MCPoint->GetAtomicNum()<<"\n";
      }
      //  }
   }
}

ClassImp(AtPSA)
