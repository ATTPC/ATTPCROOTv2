#include "../helper.h"

TH1F *hMaxLarge = new TH1F("hMaxLarge", "Maximum in large pads", 100, 450, 550);
TH1F *hMaxSmall = new TH1F("hMaxSmall", "Maximum in small pads", 100, 450, 550);
TH1F *hMaxUncalibrated = new TH1F("hMaxUncalibrated", "Max uncalibrated", 4096, 0, 4096);

void fillEvent(ULong64_t eventNumber)
{
   std::cout << "Loading event: " << eventNumber << std::endl;
   if (!loadEvent(eventNumber))
      return;
   hMaxLarge->Reset();
   hMaxSmall->Reset();
   hMaxUncalibrated->Reset();

   for (auto &pad : rawEventPtr->GetPads()) {
      auto max = *std::max_element(pad.GetADC(), pad.GetADC() + 512);
      auto rawMax = *std::max_element(pad.GetRawADC(), pad.GetRawADC() + 512);
      if (pad.GetSizeID() == 0)
         hMaxSmall->Fill(max);
      else
         hMaxLarge->Fill(max);
      hMaxUncalibrated->Fill(rawMax);
   }
}

void checkCalibration(TString fileName)
{
   loadRun(fileName);
   fillEvent(0);
}
