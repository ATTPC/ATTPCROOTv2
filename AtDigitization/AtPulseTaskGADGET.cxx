#include "AtPulseTaskGADGET.h"


#include "AtDigiPar.h"
#include "AtElectronicResponse.h"
#include "AtMCPoint.h"
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h"
#include "AtRawEvent.h"
#include "AtSimulatedPoint.h"

#include <FairLogger.h>
#include <FairParSet.h> // for FairParSet
#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <Math/Vector3D.h>
#include <TAxis.h>
#include <TClonesArray.h>
#include <TF1.h>
#include <TH1.h>
#include <TH2Poly.h>
#include <TMath.h>
#include <TObject.h>
#include <TRandom.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <utility>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

using namespace ElectronicResponse;
AtPulseTaskGADGET::AtPulseTaskGADGET() : AtPulseTaskGADGET("AtPulseTaskGADGET") {}
AtPulseTaskGADGET::AtPulseTaskGADGET(const char *name) : AtPulseTask(name) {}

   
Double_t AtPulseTaskGADGET::ChargeDispersion(Double_t G,Double_t time, Double_t x0,Double_t y0,Double_t xi,Double_t yi) { 
   Double_t rtTwoSigma = TMath::Sqrt(2*time/(R*C)+2*Dc*t_amp)*TMath::Sqrt(2)*SigmaPercent;
   Double_t Charge = G/4*(TMath::Erf((xi+W/2-x0)/(rtTwoSigma))-TMath::Erf((xi-W/2-x0)/(rtTwoSigma)))*
   (TMath::Erf((yi+W/2-y0)/(rtTwoSigma))-TMath::Erf((yi-W/2-y0)/(rtTwoSigma)));
   return Charge;
};

Int_t skippy = 0; // looking at the skipped charge dispersed electrons
bool AtPulseTaskGADGET::gatherElectronsFromSimulatedPoint(AtSimulatedPoint *point )
{  
   if (point == nullptr)
      return false;
   
   if (AdjecentPads == 0){
      SetSigmaPercent(0.01);
   };

   auto coord = point->GetPosition();
   auto xElectron = coord.x();       // mm
   auto yElectron = coord.y();       // mm
   auto eTime = coord.z();           // us
   eTime += fTBPadPlane * fTBTime;   // correct time for pad plane location
   auto charge = point->GetCharge(); // number of electrons
   auto binNumber = fPadPlane->Fill(xElectron, yElectron);
   auto padNumber = fMap->BinToPad(binNumber);
   
   if(padNumber < 0 || padNumber >= fMap->GetNumPads()){// if electron cloud hits edge center cant be calculated thus were moving the cloud an edge away.
      binNumber = fPadPlane->Fill(xElectron+0.001, yElectron+0.001);
      padNumber = fMap->BinToPad(binNumber);
      };
   
   auto mcPoint = dynamic_cast<AtMCPoint *>(fMCPointArray->At(point->GetMCPointID()));
   auto trackID = mcPoint->GetTrackID();
   auto PadCenter = fMap->CalcPadCenter(padNumber);// get pad center
   auto gAvg =  AtPulseTaskGADGET::getAvgGETgain(point->GetCharge()); // get average gain
  
// Calculate the coordinates for adjacent pads
std::vector<Double_t> coords(Items);
Int_t adjecentPads = AdjecentPads; // Capture the variable as a local copy
std::generate_n(coords.begin(), Items, [adjecentPads, n = 0]() mutable {
    Double_t coord = adjecentPads * -2.2 + 2.2 * n++;
    return coord;
});




// Iterate over the adjacent pads using the calculated coordinates
for (const auto& coord : coords) {
    Double_t xPadCurrent = PadCenter.X() + coord;
    Double_t yPadCurrent = PadCenter.Y() + coord;
         
         auto newbinNumber = fPadPlane->Fill(xPadCurrent, yPadCurrent); // assign new bin number to adjecent pad
         auto newpadNumber = fMap->BinToPad(newbinNumber); // assign new pad number to adjecent pad
   
         if (newpadNumber < 0 || newpadNumber >= fMap->GetNumPads()) {// skip if dispersped pad is invalid
            LOG(debug) << "Skipping electron...";
            skippy++;

            continue;};

         auto newtotalyInhibited = fMap->IsInhibited(newpadNumber) == AtMap::InhibitType::kTotal;
         if (!newtotalyInhibited ){
            Double_t ChargeDispersed = ChargeDispersion(gAvg,eTime ,xElectron,yElectron,xPadCurrent, yPadCurrent);// charge dispersed on adjecent pad
            // appends the charge to the histogram 
            
            eleAccumulated[newpadNumber]->Fill(eTime, ChargeDispersed);
            electronsMap[newpadNumber] = eleAccumulated[newpadNumber].get();
            
      }
   }

   if (padNumber < 0 || padNumber >= fMap->GetNumPads()) {// Skip if pad is invalid
         LOG(debug) << "Skipping electron...";
         return false;
      }
   
   return true;
}

void AtPulseTaskGADGET::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtPulseTaskGADGET";
   reset();

   Int_t nMCPoints = fSimulatedPointArray->GetEntries();
   std::cout << " AtPulseTaskGADGET: Number of Points " << nMCPoints << std::endl;
   std::cout << " AtPulseTaskGADGET: Number of Points (plus dispersion) " << (nMCPoints*Items*Items) << std::endl;
   // Distributing electron pulses among the pads
   Int_t skippedPoints = 0;
   Int_t skippedDispersion = 0;
   for (Int_t i = 0; i < nMCPoints; i++) {
      auto dElectron = dynamic_cast<AtSimulatedPoint *>(fSimulatedPointArray->At(i));
      if (dElectron == nullptr)
         LOG(fatal) << "The TClonesArray AtSimulatedPoint did not contain type AtSimulatedPoint!";
      if (!gatherElectronsFromSimulatedPoint(dElectron))
         skippedPoints++;
      skippedPoints+=skippy;
      skippedDispersion+=skippy;
      skippy=0;  
   }
   

   std::cout << "...End of collection of electrons in this event." << std::endl;
   std::cout << "Skipped " << (double)skippedPoints / (nMCPoints * Items*Items) * 100. << "% of " << (nMCPoints* Items*Items) << std::endl;
                                                         //  ^^^^^^^^^ the total pads are the number of dispersed pads times the center pads.
   std::cout << "Skipped dispersion " << (double)skippedDispersion / (nMCPoints*Items*Items) * 100  << "% of " << (nMCPoints*Items*Items) << std::endl;                                                      
   generateTracesFromGatheredElectrons();
}


ClassImp(AtPulseTaskGADGET);
