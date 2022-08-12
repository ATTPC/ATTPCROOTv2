#include <TCanvas.h>
#include <TFile.h>
#include <TH2.h>
#include <TRandom3.h>
#include <TVirtualFFT.h>
#ifndef __CLING__
#include "../build/include/AtPadFFT.h"
#include "../build/include/AtRawEvent.h"

#endif

#include "../../helper.h"

/***** "Public" functions ******/
void loadRun(int runNum);
void viewFFT(int eventNum, int padNum);
// Get the ratio of magnitude of fourier representation for hits with a charge between
// qMin and qMax
void getRatioMagnitudes(int runNum, double_t qMin = 45, double_t qMax = MAXFLOAT);
void getChargeRatio(int runNum, double_t qMin = 45, double_t qMax = MAXFLOAT);

/***** "Private" variables *****/
Int_t fCurrentRunNum = -1;
TFile *oFile = nullptr;
TCanvas *cFFT = nullptr;
TCanvas *cRatio = nullptr;
TH1F *hFilteredTrace = new TH1Fg("filteredTrace", "Filtered Trace", 512, 0, 511);
TH1F *hFilteredMag = new TH1F("filtereMag", "Filtered Magnitude", 512 / 2 + 1, 0, 512 / 2);
TH1F *hMag = new TH1F("mag", "Magnitude", 512 / 2 + 1, 0, 512 / 2);
TH1F *hRatio = new TH1F("ratio", "Ratio of Magnitudes", 512 / 2 + 1, 0, 512 / 2);
TH2F *hRatioSummary = new TH2F("ratioSummary", "|filtered|/|unfiltered|", 512 / 2 + 1, 0, 512 / 2, 200, 0, 2);
TH2F *hRatioPhase = new TH2F("ratioPhase", "Arg(filtered)/Arg(unfiltered)", 512 / 2 + 1, 0, 512 / 2, 100, -2, 2);
TH1F *hQRatio = new TH1F("hQRatio", "Ratio between max adc", 1000, 0, 1000);
TH1F *hQ = new TH1F("hQ", "Q", 1000, 0, 4092);
TH1F *hQRaw = new TH1F("hQRaw", "Q Raw", 1000, 0, 4092);
/******** Implementation ********/
void fftDev(int runNum = 195)
{
   // loadRun(runNum);
}
void getChargePad(int runNum, int padNum)
{
   loadRun(runNum);
   hQ->Reset();
   hQRaw->Reset();
   while (nextEvent()) {
      if (reader->GetCurrentEntry() % 1 == 0)
         std::cout << "Entry: " << reader->GetCurrentEntry() << " of " << reader->GetEntries() << std::endl;
      if (reader->GetCurrentEntry() > 100)
         break;

      for (auto &hit : eventPtr->GetHitArray())
         if (hit.GetPadNum() == padNum) {
            auto padRaw = rawEventPtr->GetPad(hit.GetPadNum());
            auto pad = rawEventFilteredPtr->GetPad(hit.GetPadNum());
            auto qRaw = *std::max_element(padRaw->GetADC().begin(), padRaw->GetADC().end());
            auto q = *std::max_element(pad->GetADC().begin(), pad->GetADC().end());
            if (q != qRaw) {
               hQ->Fill(q);
               hQRaw->Fill(qRaw);
            }
         }
   }
   cFFT->cd(1);
   hQ->Draw();
   cFFT->cd(3);
   hQRaw->Draw();
   oFile->cd();
   hQ->Write();
   hQRaw->Write();
}

void getChargeRatio(int runNum, double_t qMin, double_t qMax)
{
   loadRun(runNum);
   hQRatio->Reset();
   hQ->Reset();
   hQRaw->Reset();
   while (nextEvent()) {
      if (reader->GetCurrentEntry() % 100 == 0)
         std::cout << "Entry: " << reader->GetCurrentEntry() << " of " << reader->GetEntries() << std::endl;
      if (reader->GetCurrentEntry() > 1000000)
         break;

      for (auto &hit : eventPtr->GetHitArray())
         if (hit.GetCharge() >= qMin && hit.GetCharge() <= qMax) {
            auto padRaw = rawEventPtr->GetPad(hit.GetPadNum());
            auto pad = rawEventFilteredPtr->GetPad(hit.GetPadNum());
            auto qRaw = *std::max_element(padRaw->GetADC().begin(), padRaw->GetADC().end());
            auto q = *std::max_element(pad->GetADC().begin(), pad->GetADC().end());
            if (q != qRaw) {
               hQRatio->Fill(q / qRaw);
               hQ->Fill(q);
               hQRaw->Fill(qRaw);
            }
         }
   }
   cRatio->cd();
   hQRatio->Draw();
   cFFT->cd(1);
   hQ->Draw();
   cFFT->cd(3);
   hQRaw->Draw();
   oFile->cd();
   hQRatio->Write();
   hQ->Write();
   hQRaw->Write();
}
void getRatioMagnitudes(int runNum, double_t qMin, double_t qMax)
{
   loadRun(runNum);
   hRatioSummary->Clear();
   hRatioPhase->Clear();
   while (nextEvent()) {
      if (reader->GetCurrentEntry() % 100 == 0)
         std::cout << "Entry: " << reader->GetCurrentEntry() << " of " << reader->GetEntries() << std::endl;
      if (reader->GetCurrentEntry() > 10000)
         break;
      for (auto &hit : eventPtr->GetHitArray())
         if (hit.GetCharge() >= qMin && hit.GetCharge() <= qMax) {
            auto padRaw = dynamic_cast<AtPadFFT *>(rawEventPtr->GetPad(hit.GetPadNum()));
            auto pad = dynamic_cast<AtPadFFT *>(rawEventFilteredPtr->GetPad(hit.GetPadNum()));
            for (int i = 0; i < 512 / 2 + 1; ++i) {
               auto ratio = pad->GetPointMag(i) / padRaw->GetPointMag(i);
               auto ratioPhase = pad->GetPointPhase(i) / padRaw->GetPointPhase(i);
               if (ratio == 1)
                  continue;
               hRatioSummary->Fill(i, ratio);
               hRatioPhase->Fill(i, ratioPhase);
            }
         }
   }
   std::cout << "Done processing tree" << std::endl;
   cRatio->SetLogz();
   // hRatioSummary->Draw("colz");
   hRatioPhase->Draw("colz");
   // cRatio->cd(1);
   // cRatio->cd(2);
   // hRatioSummaryInverse->Draw("colz");
}

void loadRun(int runNum)
{
   if (runNum == fCurrentRunNum)
      return;

   TString filePath = "/mnt/analysis/e12014/TPC/filterTesting/run_%04d.root";
   loadRun(TString::Format(filePath, runNum), "AtRawEvent", "AtRawEventFFTRaw", "AtEventH");
   loadEvent(0);

   if (dynamic_cast<AtPadFFT *>(rawEventPtr->GetPads().back().get()) == nullptr)
      LOG(error) << "Raw event branch does not contain FFT pads!";
   if (dynamic_cast<AtPadFFT *>(rawEventFilteredPtr->GetPads().back().get()) == nullptr)
      LOG(error) << "Filtered raw event branch does not contain FFT pads!";

   if (oFile != nullptr)
      delete oFile;
   oFile = new TFile(TString::Format("output/histo-%d.root", runNum), "RECREATE");

   if (cFFT == nullptr) {
      cFFT = new TCanvas("cFFT", "FFT", 900, 900);
      cFFT->Divide(2, 2);
   }
   if (cRatio == nullptr) {
      cRatio = new TCanvas("cRatio", "Ratio", 600, 400);
      // cRatio->Divide(1, 2);
   }
   fCurrentRunNum = runNum;
}

void viewFFT(int eventNum, int padNum)
{
   if (!loadEvent(eventNum))
      return;
   if (!loadPad(padNum))
      return;

   auto pad = dynamic_cast<AtPadFFT *>(rawEventPtr->GetPad(padNum));
   auto padFiltered = dynamic_cast<AtPadFFT *>(rawEventFilteredPtr->GetPad(padNum));

   for (int i = 0; i < 512 / 2 + 1; ++i) {
      auto point = pad->GetPoint(i);
      auto mag = std::sqrt(point.first * point.first + point.second * point.second);
      hMag->SetBinContent(i + 1, mag);

      auto pointFilter = padFiltered->GetPoint(i);
      auto magFilter = std::sqrt(pointFilter.first * pointFilter.first + pointFilter.second * pointFilter.second);
      hFilteredMag->SetBinContent(i + 1, magFilter);

      hRatio->SetBinContent(i + 1, magFilter / mag);
   }

   for (int i = 0; i < 512; ++i)
      hFilteredTrace->SetBinContent(i + 1, padFiltered->GetADC(i));

   cFFT->cd(1);
   hTrace->Draw();
   cFFT->cd(2);
   hFilteredTrace->Draw();
   cFFT->cd(3);
   hMag->Draw();
   cFFT->cd(4);
   hFilteredMag->Draw();
   cRatio->cd();
   hRatio->Draw();
}
