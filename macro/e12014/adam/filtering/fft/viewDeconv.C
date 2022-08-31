
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
#include "../../helper.h"

TCanvas *cConv = nullptr;
TCanvas *cDiff = nullptr;
TCanvas *cPad = nullptr;

std::vector<TH1D *> hCharge;
std::array<TH1D *, 2> hFFq;
std::array<TH1D *, 2> hFFqReco;
std::array<TGraph *, 2> grQratio;

void viewDeconv()
{
   if (cConv == nullptr) {
      cConv = new TCanvas("cConv", "Canvas", 600, 600);
      cConv->Divide(2, 2);
   }
   if (cDiff == nullptr) {
      cDiff = new TCanvas("cDiff", "Charge ratio", 600, 300);
      cDiff->Divide(2, 1);
   }
   if (cPad == nullptr) {
      cPad = new TCanvas("cPad", "Pad", 300, 600);
      cPad->Divide(1, 2);
   }

   loadRun("../../../simulation/eventGenerator/sym90/output_digi.root");
}

void viewDeconv(int eventNum, int padNum)
{
   loadEvent(eventNum);
   auto pad = rawEventPtr->GetPad(padNum);
   if (pad == nullptr) {
      LOG(error) << "Pad number " << padNum << " is not in event " << eventNum;
      return;
   }

   for (auto &[name, ptr] : pad->GetAugments())
      LOG(info) << name << " " << ptr.get();

   // Get the Q and Q reco
   auto &q = dynamic_cast<AtPadArray *>(pad->GetAugment("Q"))->GetArray();
   auto &Reco = dynamic_cast<AtPadArray *>(pad->GetAugment("Qreco"))->GetArray();

   while (hCharge.size() < 2)
      hCharge.push_back(new TH1D(TString::Format("hQ%lu", hCharge.size()), "Charge", 512, 0, 512));

   ContainerManip::SetHistFromData(*hCharge[0], q);
   ContainerManip::SetHistFromData(*hCharge[1], Reco);

   cPad->cd(1);
   hCharge[0]->Draw("hist");
   cPad->cd(2);
   hCharge[1]->Draw("hist");
}

void viewDeconv(int eventNum, bool absDiff = true)
{
   loadEvent(eventNum);
   if (hFFq.back() == nullptr) {
      hFFq[0] = new TH1D("hFFq0", "FF 1 Charge", 512, 0, 512);
      hFFq[1] = new TH1D("hFFq1", "FF 2 Charge", 512, 0, 512);
      hFFqReco[0] = new TH1D("hFFqReco0", "FF 1 Charge Reconstructed", 512, 0, 512);
      hFFqReco[1] = new TH1D("hFFqReco1", "FF 2 Charge Reconstructed", 512, 0, 512);
   }

   if (patternEventPtr->GetTrackCand().size() < 2) {
      LOG(error) << "Not enough tracks to try creating charge distrobutions!";
      return;
   }

   for (int i = 0; i < 2; ++i) {
      hFFq[i]->Reset();
      hFFqReco[i]->Reset();

      // Get the ransac track for a FF and construct charge plots
      auto track = patternEventPtr->GetTrackCand()[i];

      for (auto &hit : track.GetHitArrayConst()) {
         auto pad = rawEventPtr->GetPad(hit.GetPadNum());
         auto &q = dynamic_cast<AtPadArray *>(pad->GetAugment("Q"))->GetArray();
         auto &Reco = dynamic_cast<AtPadArray *>(pad->GetAugment("Qreco"))->GetArray();

         // Fill the histograms
         hFFq[i]->Add(ContainerManip::CreateHistFromData("temp", q).get());
         hFFqReco[i]->Add(ContainerManip::CreateHistFromData("temp2", Reco).get());
      }

      double norm = std::accumulate(hFFq[i]->GetArray() + 100, hFFq[i]->GetArray() + 200, 0.0) /
                    std::accumulate(hFFqReco[i]->GetArray() + 100, hFFqReco[i]->GetArray() + 200, 0.0);

      cConv->cd(i + 1);
      hFFq[i]->Draw("hist");
      cConv->cd(i + 3);
      hFFqReco[i]->Draw("hist");

      std::array<double, 512> ratio;
      std::array<double, 512> x;
      for (int j = 0; j < 512; ++j) {
         x[j] = j;

         if (absDiff)
            ratio[j] = (hFFq[i]->GetBinContent(j + 1) - (norm * hFFqReco[i]->GetBinContent(j + 1)));
         else {
            if (hFFq[i]->GetBinContent(j + 1) != 0)
               ratio[j] = (hFFq[i]->GetBinContent(j + 1) - (norm * hFFqReco[i]->GetBinContent(j + 1))) /
                          hFFq[i]->GetBinContent(j + 1);
            else
               ratio[j] = 0;
         }
      }

      if (grQratio[i] != nullptr)
         delete grQratio[i];
      grQratio[i] = new TGraph(x.size(), x.data(), ratio.data());
      cDiff->cd(i + 1);
      grQratio[i]->Draw("ACP");
   } // End loop over i
}
