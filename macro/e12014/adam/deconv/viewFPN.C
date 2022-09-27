
#include <TCanvas.h>
#include <TComplex.h>
#include <TF1.h>
#include <TFile.h>
#include <TFitResult.h>
#include <TFitResultPtr.h>
#include <TGraph.h>
#include <TGraphErrors.h>
#include <TH2.h>
#include <TRandom3.h>
#include <TVirtualFFT.h>

#include <fstream>
#include <numeric>

#ifndef __CLING__
#include <FairParAsciiFileIo.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

#include "../build/include/AtContainerManip.h"
#include "../build/include/AtDigiPar.h"
#include "../build/include/AtHit.h"
#include "../build/include/AtPSADeconv.h"
#include "../build/include/AtPadArray.h"
#include "../build/include/AtPadFFT.h"
#include "../build/include/AtPulserInfo.h"
#include "../build/include/AtRawEvent.h"
#endif
#include "../helper.h"

TCanvas *cTrace = nullptr;
std::unique_ptr<TH1D> trace;
std::unique_ptr<TH1D> fpn;
std::unique_ptr<TH1D> traceFill;
std::unique_ptr<TH1D> fpnFill;

void viewFPN(int runNum = 29)
{
   if (cTrace == nullptr) {
      cTrace = new TCanvas("cTrace", "Trace", 900, 600);
      cTrace->Divide(2, 2);
   }

   // loadRun(TString::Format("./run_%04d.root", runNum));
   // RawEvent: NoPulserData
   // loadRun(TString::Format("./run_%04dSub.root", runNum), "NoPulserData");
   loadRun(TString::Format("./run_%04dSubAget.root", runNum), "NoPulserData");
}

void viewFPN(int eventNum, AtPadReference ref)
{
   loadEvent(eventNum);

   auto fpnPad = rawEventPtr->GetFpn(ref);
   fpn = fpnPad->GetADCHistrogram();
   fpnFill = rawEventFilteredPtr->GetFpn(ref)->GetADCHistrogram();

   cTrace->cd(2);
   fpn->Draw();
   cTrace->cd(4);
   fpnFill->Draw();

   auto fpnFilledPad = rawEventFilteredPtr->GetFpn(ref);

   for (auto &[name, ptr] : fpnFilledPad->GetAugments())
      std::cout << name << " " << ptr.get() << std::endl;

   auto pulserInfo = dynamic_cast<AtPulserInfo *>(fpnFilledPad->GetAugment("pulserInfo"));
   if (pulserInfo != nullptr) {
      std::cout << pulserInfo->GetBegin()[0] << " " << pulserInfo->GetEnd()[0] << " " << pulserInfo->GetMag()[0]
                << std::endl;
      std::cout << pulserInfo->GetBegin()[1] << " " << pulserInfo->GetEnd()[1] << " " << pulserInfo->GetMag()[1]
                << std::endl;
   }
}

void viewFPN(int eventNum, int padNum)
{
   loadEvent(eventNum);
   auto pad = rawEventPtr->GetPad(padNum);
   if (pad == nullptr) {
      LOG(error) << "Pad number " << padNum << " is not in event " << eventNum;
      return;
   }

   std::cout << "For " << tpcMap->GetPadRef(padNum) << " getting FPN " << tpcMap->GetNearestFPN(padNum) << std::endl;
   viewFPN(eventNum, tpcMap->GetNearestFPN(padNum));

   trace = pad->GetADCHistrogram();
   traceFill = rawEventFilteredPtr->GetPad(padNum)->GetADCHistrogram();

   cTrace->cd(1);
   trace->Draw();
   cTrace->cd(3);
   traceFill->Draw();
}
