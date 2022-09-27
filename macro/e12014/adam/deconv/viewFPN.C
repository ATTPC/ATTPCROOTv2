
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
#include "../build/include/AtRawEvent.h"
#endif
#include "../helper.h"

TCanvas *cTrace = nullptr;
std::unique_ptr<TH1D> trace;
std::unique_ptr<TH1D> fpn;

void viewFPN(int runNum = 29)
{
   if (cTrace == nullptr) {
      cTrace = new TCanvas("cTrace", "Trace", 600, 600);
      cTrace->Divide(1, 2);
   }

   loadRun(TString::Format("./run_%04d.root", runNum));
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
   auto fpnPad = rawEventPtr->GetFpn(tpcMap->GetNearestFPN(padNum));

   trace = pad->GetADCHistrogram();
   fpn = fpnPad->GetADCHistrogram();

   cTrace->cd(1);
   trace->Draw();
   cTrace->cd(2);
   fpn->Draw();
}
