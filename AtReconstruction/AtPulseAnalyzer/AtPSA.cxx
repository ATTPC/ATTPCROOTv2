#include "AtPSA.h"

#include "AtDigiPar.h"
#include "AtEvent.h"
#include "AtHit.h"
#include "AtMCPoint.h"
#include "AtPad.h"
#include "AtRawEvent.h"

#include <FairLogger.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <Math/Point3D.h> // for PositionVector3D
#include <Rtypes.h>
#include <TClonesArray.h>
#include <TObject.h> // for TObject

#include <algorithm>
#include <array> // for array
#include <cmath> // for pow
#include <iostream>
#include <iterator>
#include <utility> // for pair

using std::distance;
using std::max_element;
using std::min_element;

void AtPSA::Init()
{

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb(); // NOLINT
   if (!db)
      LOG(fatal) << "No runtime database!";

   auto fPar = (AtDigiPar *)db->getContainer("AtDigiPar"); // NOLINT
   if (!fPar) {
      LOG(fatal) << "AtDigiPar not found!!";
      return;
   }

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
   return (fNumTbs - peakIdx) * fTBTime * fDriftVelocity / 100.;
}

Double_t AtPSA::CalculateZGeo(Double_t peakIdx)
{
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

AtEvent AtPSA::Analyze(AtRawEvent &rawEvent)
{
   AtEvent event;
   Analyze(&rawEvent, &event);
   return event;
}

/**
 * Analyzes every pad in event, and add the hits to AtEvent. Called by AtPSATask.
 *
 */
void AtPSA::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{
   Double_t QEventTot = 0.0;
   Double_t RhoVariance = 0.0;
   Double_t RhoMean = 0.0;
   Double_t Rho2 = 0.0;

   std::map<Int_t, Int_t> PadMultiplicity;
   std::array<Float_t, 512> mesh{};
   mesh.fill(0);

   for (const auto &pad : rawEvent->GetPads()) {

      LOG(debug) << "Running PSA on pad " << pad->GetPadNum();

      auto hits = AnalyzePad(pad.get());

      PadMultiplicity.insert(std::pair<Int_t, Int_t>(pad->GetPadNum(), hits.size()));

      // If we got any hits update the mesh signal
      if (hits.size() != 0)
         for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
            mesh[iTb] += pad->GetADC(iTb);

      // Update AtEvent with hits
      for (auto &&hit : hits) {
         auto pos = hit->GetPosition();
         QEventTot += hit->GetTraceIntegral();
         Rho2 += pos.Mag2();
         RhoMean += pos.Rho();

         auto mcPointsMap = rawEvent->GetSimMCPointMap();
         if (mcPointsMap.size() > 0) {
            LOG(debug) << "MC Simulated points Map size " << mcPointsMap.size();
            TrackMCPoints(mcPointsMap, *(hit.get()));
         }

         event->AddHit(std::move(hit));
      }
   }

   // RhoVariance = Rho2 - (pow(RhoMean, 2) / (event->GetNumHits()));
   RhoVariance = Rho2 - (event->GetNumHits() * pow((RhoMean / event->GetNumHits()), 2));

   for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
      event->SetMeshSignal(iTb, mesh[iTb]);
   event->SortHitArrayTime();
   event->SetMultiplicityMap(PadMultiplicity);
   event->SetRhoVariance(RhoVariance);
   event->SetEventCharge(QEventTot);
}

Double_t AtPSA::getThreshold(int padSize)
{
   if (padSize == 0)
      return fThresholdlow; // threshold for central pads
   return fThreshold;       // threshold for big pads (or all other not small)
}

/**
 * Returns a vector populated with the variance of the hit according the the diffusion
 * coefficients in the parameter file.
 *
 * @param[in] zLoc Location of hit in TB
 * @param[in] zLocVar Variance of hit in TB^2
 * @return variance of hit location in mm^2
 * @todo Implement the function so it doesn't return all zeros...
 */
double AtPSA::getZhitVariance(double zLoc, double zLocVar) const
{
   return 0;
}

std::pair<double, double> AtPSA::getXYhitVariance() const
{
   return {0, 0};
}

ClassImp(AtPSA)
