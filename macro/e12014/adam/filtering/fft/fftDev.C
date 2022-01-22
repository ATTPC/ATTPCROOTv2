#include "../../helper.h"
#include "TVirtualFFT.h"

/***** "Public" functions ******/
void loadRun(int runNum);
void runFFT(int padNum);

/***** "Private" variables *****/
TFile *oFile = nullptr;

void loadRun(int runNum)
{
   TString filePath = "/mnt/analysis/e12014/TPC/unpacked/run_%04d.root";
   loadRun(TString::Format(filePath, runNum), "AtRawEvent", "AtEventH");

   if(oFile != nullptr)
      delete oFile;
   oFile = new TFile(TString::Format("output/traces-%d.root", runNum), "RECREATE");
}

void runFFT(int padNum)
{
   loadEvent(0);
   loadPad(padNum);
   oFile->cd();
   hTrace->Write();
   return;

   
   hTrace->Draw();
   TH1 *hTrans = nullptr;
   TVirtualFFT::SetTransform(nullptr);
   hTrans = hTrace->FFT(hTrans, "MAG");
   hTrans->SetTitle("Magnitude of 1st transform");

   TCanvas *c2 = new TCanvas();
   hTrans->Draw();
}
