#include "AtPulseGADGET.h"

#include "AtElectronicResponse.h"
#include "AtMap.h"
#include "AtRawEvent.h"
#include "AtSimulatedPoint.h"

#include <FairLogger.h>

#include <Math/Point2D.h>    // for PositionVector2D
#include <Math/Point2Dfwd.h> // for XYPoint
#include <Math/Vector3D.h>
#include <TH1.h>
#include <TMath.h>

#include <iostream>
#include <set> // for set
class AtPad;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
using XYPoint = ROOT::Math::XYPoint;

using namespace ElectronicResponse;
AtPulseGADGET::AtPulseGADGET(AtMapPtr map) : AtPulse(map) {}

Double_t AtPulseGADGET::ChargeDispersion(Double_t G, Double_t time, Double_t x0, Double_t y0, Double_t xi, Double_t yi)
{
   Double_t Sigma = TMath::Sqrt(2 * time / (R * C) + 2 * Dc * t_amp) * SigmaPercent;
   Double_t rtTwo = TMath::Sqrt(2);
   Double_t Charge =
      G / 4 * (TMath::Erf((xi + W / 2 - x0) / (rtTwo * Sigma)) - TMath::Erf((xi - W / 2 - x0) / (rtTwo * Sigma))) *
      (TMath::Erf((yi + W / 2 - y0) / (rtTwo * Sigma)) - TMath::Erf((yi - W / 2 - y0) / (rtTwo * Sigma)));
   return Charge;
};

bool AtPulseGADGET::AssignElectronsToPad(AtSimulatedPoint *point)
{
   fSkippy = 0;

   if (point == nullptr)
      return false;

   if (AdjecentPads == 0) {
      SetSigmaPercent(0.01);
   };

   auto coord = point->GetPosition();
   auto xElectron = coord.x();     // mm
   auto yElectron = coord.y();     // mm
   auto eTime = coord.z();         // us
   eTime += fTBPadPlane * fTBTime; // correct time for pad plane location
   auto padNumber = fMap->GetPadNum(XYPoint{xElectron, yElectron});

   if (padNumber < 0 || padNumber >= fMap->GetNumPads()) { // if electron cloud hits edge center cant be calculated thus
                                                           // were moving the cloud an edge away.
      padNumber = fMap->GetPadNum(XYPoint{xElectron + 0.001, yElectron + 0.001});
   };

   auto PadCenter = fMap->CalcPadCenter(padNumber); // get pad center

   // Setting up cordinates for adjecent pads
   Double_t coords[Items];
   for (Int_t n = 0; n < Items; n++) {
      coords[n] = AdjecentPads * -2.2 + 2.2 * n;
   };
   // Store the pad range in a variable
   Int_t numPads = fMap->GetNumPads();

   // Calculate xPadCurrent and yPadCurrent outside the loop
   Double_t xPadCurrent, yPadCurrent;
   for (Int_t i = 0; i < Items; i++) {
      xPadCurrent = PadCenter.X() + coords[i]; // NOLINT
      for (Int_t j = 0; j < Items; j++) {
         yPadCurrent = PadCenter.Y() + coords[j]; // NOLINT

         // Calculate newpadNumber directly from newbinNumber
         auto newpadNumber = fMap->GetPadNum(XYPoint{xPadCurrent, yPadCurrent});
         auto gAvg = GetGain(newpadNumber, point->GetCharge()); // get average gain

         if (newpadNumber < 0 || newpadNumber >= numPads || gAvg == 0) {
            LOG(debug) << "Skipping electron...";
            fSkippy++;
            continue;
         }

         Double_t ChargeDispersed = ChargeDispersion(gAvg, eTime, xElectron, yElectron, xPadCurrent, yPadCurrent);
         fPadCharge[newpadNumber]->Fill(eTime, ChargeDispersed);
         fPadsWithCharge.insert(newpadNumber);
      }
   }

   if (padNumber < 0 || padNumber >= numPads) {
      LOG(debug) << "Skipping electron...";
      return false;
   }

   return true;
}

AtRawEvent AtPulseGADGET::GenerateEvent(std::vector<AtSimulatedPoint *> &vec)
{
   LOG(debug) << "Exec of AtPulseGADGET";
   Reset();

   Int_t nMCPoints = vec.size();
   std::cout << " AtPulseGADGET: Number of Points " << nMCPoints << std::endl;
   std::cout << " AtPulseGADGET: Number of Points (plus dispersion) " << (nMCPoints * Items * Items) << std::endl;
   // Distributing electron pulses among the pads

   int numFilled = 0;
   int skippedDispersion = 0;
   for (auto &point : vec) {
      numFilled += AssignElectronsToPad(point);
      numFilled -= fSkippy;
      skippedDispersion += fSkippy;
   }
   LOG(info) << "Skipped " << (double)(vec.size() - numFilled) / vec.size() * 100 << "% of " << vec.size()
             << " points.";

   LOG(info) << "Skipped dispersion " << (double)skippedDispersion / (nMCPoints * Items * Items) * 100 << "% of "
             << (nMCPoints * Items * Items);

   AtRawEvent ret;
   for (auto padNum : fPadsWithCharge) {
      AtPad *pad = ret.AddPad(padNum);
      FillPad(*pad, *fPadCharge[padNum]);
   }
   return ret;
}
