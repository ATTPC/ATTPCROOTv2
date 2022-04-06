#include "AtPSASimple.h"

#include <RtypesCore.h>
#include <Math/Point2D.h>
// STL
#include <algorithm>
#include <array>
#include <iterator>
#include <memory>

// AtTPCROOT classes
#include "AtRawEvent.h"
#include "AtEvent.h"
#include "AtHit.h"
#include "AtPad.h"
#include <Rtypes.h>

using std::distance;
using std::max_element;
using std::min_element;

ClassImp(AtPSASimple) AtPSASimple::AtPSASimple() {}

AtPSASimple::~AtPSASimple() {}

void AtPSASimple::Analyze(AtRawEvent *rawEvent, AtEvent *event)
{
   Int_t numPads = rawEvent->GetNumPads();

   for (Int_t iPad = 0; iPad < numPads; iPad++) {
      const AtPad *pad = rawEvent->GetPads().at(iPad).get();

      auto pos = pad->GetPadCoord();
      Double_t zPos = 0;
      Double_t charge = 0;

      if (pad->IsPedestalSubtracted()) { // TODO after pedestal subtraction
         auto adc = pad->GetADC();
         Int_t maxAdcIdx = distance(adc.begin(), max_element(adc.begin() + 4, adc.begin() + fNumTbs - 5));

         zPos = CalculateY(maxAdcIdx);
         charge = adc[maxAdcIdx];

         if (fThreshold > 0 && charge < fThreshold)
            continue;
      } else {
         auto rawAdc = pad->GetRawADC();
         Int_t minAdcIdx = distance(rawAdc.begin(), min_element(rawAdc.begin() + 4, rawAdc.begin() + fNumTbs - 5));

         zPos = CalculateZ(minAdcIdx);
         charge = rawAdc[minAdcIdx];

         if (fThreshold > 0 && charge > fThreshold)
            continue;
      }

      event->AddHit(pad->GetPadNum(), XYZPoint(pos.X(), pos.Y(), zPos), charge);
   }
}
