#include "AtPSASimple.h"

// AtTPCROOT classes
#include "AtRawEvent.h"
#include "AtEvent.h"
#include "AtDigiPar.h"
#include "AtCalibration.h"
#include "AtHit.h"

// ROOT classes
#include "TH1F.h"

// STL
#include <algorithm>

using std::distance;
using std::max_element;
using std::min_element;

ClassImp(AtPSASimple) AtPSASimple::AtPSASimple() {}

AtPSASimple::~AtPSASimple() {}

void AtPSASimple::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{
   Int_t numPads = rawEvent->GetNumPads();

   for (Int_t iPad = 0; iPad < numPads; iPad++) {
      AtPad *pad = &(rawEvent->GetPads().at(iPad));

      Double_t xPos = pad->GetPadXCoord();
      Double_t yPos = pad->GetPadYCoord();
      Double_t zPos = 0;
      Double_t charge = 0;

      if (pad->IsPedestalSubtracted()) { // TODO after pedestal subtraction
         Double_t *adc = pad->GetADC();
         Int_t maxAdcIdx = distance(adc, max_element(adc + 4, adc + fNumTbs - 5));

         zPos = CalculateY(maxAdcIdx);
         charge = adc[maxAdcIdx];

         if (fThreshold > 0 && charge < fThreshold)
            continue;
      } else {
         Int_t *rawAdc = pad->GetRawADC();
         Int_t minAdcIdx = distance(rawAdc, min_element(rawAdc + 4, rawAdc + fNumTbs - 5));

         zPos = CalculateZ(minAdcIdx);
         charge = rawAdc[minAdcIdx];

         if (fThreshold > 0 && charge > fThreshold)
            continue;
      }

      event->AddHit(pad->GetPadNum(), XYZPoint(xPos, yPos, zPos), charge);
   }
}
