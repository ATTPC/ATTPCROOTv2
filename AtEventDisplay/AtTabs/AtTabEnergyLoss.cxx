#include "AtTabEnergyLoss.h"

#include "AtCSVReader.h"            // for CSVRow, CSVRange, CSVIterator
#include "AtDataManip.h"            // for GetTB
#include "AtE12014.h"               // for E12014
#include "AtHit.h"                  // for AtHit, AtHit::XYZPoint, AtHit::X...
#include "AtMap.h"                  // for AtMap, AtMap::InhibitType, AtMap...
#include "AtRawEvent.h"             // for AtRawEvent
#include "AtViewerManager.h"        // for AtViewerManager
#include "AtViewerManagerSubject.h" // for AtTreeEntry, AtBranch

#include <FairLogger.h> // for Logger, LOG

#include <TCanvas.h> // for TCanvas
#include <TF1.h>     // for TF1, TF1::EAddToList, TF1::EAddT...
#include <TH1.h>     // for TH1F
#include <THStack.h> // for THStack
#include <TMath.h>   // for Sqrt, Sq, CeilNint
#include <TString.h> // for TString, operator<<

#include <algorithm> // for max
#include <cmath>     // for abs, sqrt
#include <iostream>  // for ifstream
#include <string>    // for getline, string

using XYZVector = ROOT::Math::XYZVector;
using XYZPoint = ROOT::Math::XYZPoint;

void AtTabEnergyLoss::SetStyle(std::array<TH1Ptr, 2> &hists, THStack &stack)
{
   for (int i = 0; i < hists.size(); ++i) {
      hists[i]->SetLineColor(fHistColors[i]);
      hists[i]->SetMarkerColor(fHistColors[i]);
      hists[i]->SetMarkerStyle(21);
      hists[i]->SetDirectory(nullptr);

      stack.Add(hists[i].get());
   }
}

AtTabEnergyLoss::AtTabEnergyLoss(DataHandling::AtBranch &fissionBranch)
   : AtTabCanvas("dE/dx", 2, 2), fRawEvent(AtViewerManager::Instance()->GetRawEventBranch()),
     fFissionEvent(fissionBranch), fEntry(AtViewerManager::Instance()->GetCurrentEntry()), fBinWidth(100),
     fSigmaFromHit(2), fTBtoAvg(10), fRatioFunc(std::make_unique<TF1>("ratioFunc", "pol0", 0, 1, TF1::EAddToList::kNo)),
     fProxyFunc(std::make_unique<TF1>("proxyFunc", "pol0", 0, 1, TF1::EAddToList::kNo)),
     fZFunc(std::make_unique<TF1>("zFunc", "pol0", 0, 1, TF1::EAddToList::kNo))
{

   double maxLength = TMath::Sqrt(TMath::Sq(1000) + TMath::Sq(25));
   int nBins = TMath::CeilNint(maxLength / fBinWidth.Get());

   // Want bins = number of TB across detector
   int minBin = 0;
   int numBins = 512; // AtTools::GetDriftTB(1000);
   int maxBin = 512;  // 1000;

   dEdx[0] = std::make_unique<TH1F>("dEdx1", "dEdX Frag 1", nBins, 0, maxLength);
   dEdx[1] = std::make_unique<TH1F>("dEdx2", "dEdX Frag 2", nBins, 0, maxLength);
   SetStyle(dEdx, dEdxStack);

   dEdxZ[0] = std::make_unique<TH1F>("dEdxZ1", "dEdX Frag 1", nBins, 0, maxLength);
   dEdxZ[1] = std::make_unique<TH1F>("dEdxZ2", "dEdX Frag 2", nBins, 0, maxLength);
   SetStyle(dEdxZ, dEdxStackZ);

   /** Sum of Q information */
   // fSumQ[0] = std::make_unique<TH1F>("qSum_0", "Q Sum Frag 1", 512, 0, 512);
   // fSumQ[1] = std::make_unique<TH1F>("qSum_1", "Q Sum Frag 2", 512, 0, 512);
   fSumQ[0] = std::make_unique<TH1F>("qSum_0", "Q Sum Frag 1", numBins, minBin, maxBin);
   fSumQ[1] = std::make_unique<TH1F>("qSum_1", "Q Sum Frag 2", numBins, minBin, maxBin);
   SetStyle(fSumQ, dEdxStackSum);

   fSumFit[0] = std::make_unique<TH1F>("fitSum_0", "Fit Sum Frag 1", numBins, minBin, maxBin);
   fSumFit[1] = std::make_unique<TH1F>("fitSum_1", "Fit Sum Frag 2", numBins, minBin, maxBin);
   SetStyle(fSumFit, dEdxStackFit);

   fRatioQ = std::make_unique<TH1F>("ratioQ", "Ratio of Q Sum", numBins, minBin, maxBin);
   fRatioFit = std::make_unique<TH1F>("ratioFit", "Ratio of Fit Sum", numBins, minBin, maxBin);
   fProxy = std::make_unique<TH1F>("proxy", "Z Proxy", numBins, minBin, maxBin);
   fZHist = std::make_unique<TH1F>("zHist", "Z of light fragment", numBins, minBin, maxBin);

   fVetoPads = {{0, 1, 1, 6},  {0, 1, 1, 7},  {0, 1, 1, 9},  {0, 1, 1, 10}, {0, 1, 1, 12}, {0, 1, 1, 39},
                {0, 1, 1, 40}, {0, 1, 1, 41}, {0, 1, 1, 44}, {0, 1, 1, 43}, {0, 1, 1, 46}, {0, 1, 3, 13}};

   std::ifstream file("/mnt/projects/hira/e12014/tpcSharedInfo/e12014_zap.csv");
   if (!file.is_open())
      LOG(fatal) << "File not open";

   std::string header;
   std::getline(file, header);

   for (auto &row : CSVRange<int>(file)) {
      fVetoPads.push_back({row[0], row[1], row[2], row[3]});
   }

   fEntry.Attach(this);
}

AtTabEnergyLoss::~AtTabEnergyLoss()
{
   fEntry.Detach(this);
}

void AtTabEnergyLoss::InitTab() {}

void AtTabEnergyLoss::Update(DataHandling::AtSubject *sub)
{
   if (sub == &fEntry) {
      Update();
      UpdateCanvas();
   }
}

void AtTabEnergyLoss::Update()
{
   for (auto &hist : dEdx)
      hist->Reset();
   for (auto &hist : dEdxZ)
      hist->Reset();
   for (auto &hist : fSumQ)
      hist->Reset();
   for (auto &hist : fSumFit)
      hist->Reset();
   fRatioQ->Reset();
   fRatioFit->Reset();
   fProxy->Reset();
   fZHist->Reset();

   if (fFissionEvent.Get() == nullptr) {
      LOG(info) << "Cannot find AtFissionEvent in branch " << fFissionEvent.GetBranch().GetBranchName();
      return;
   }

   setdEdX();

   // Fill fSumQ and fSumFit
   FillSums();
   FillRatio();

   fCanvas->cd(1);
   dEdxStackSum.Draw("nostack;hist");
   fCanvas->cd(2);
   dEdxStackFit.Draw("nostack;hist");

   // dEdxStack.Draw("nostack,X0,ep1");
   // dEdxStackZ.Draw("nostack,X0,ep1");

   fCanvas->cd(3);
   fZHist->Draw("hist");
   fZFunc->Draw("SAME");
   // fRatioFunc->Draw("SAME");

   fCanvas->cd(4);
   fRatioFit->Draw("hist");
   fRatioFunc->Draw("SAME");
}

float AtTabEnergyLoss::GetZ(int Zcn, float R)
{
   return Zcn / (2 * R) * (R - 1 + std::sqrt(1 - R * R));
}
void AtTabEnergyLoss::FillRatio()
{
   // Get the hit to use as the start of the ratio filling. We want the index that is closest to
   // the pad plane (location is closest to zero)
   int index = (fTrackStart[0] < fTrackStart[1]) ? 0 : 1;
   int tbFinal = AtTools::GetTB(fTrackStart[index]);
   int tbIni = tbFinal - fTBtoAvg.Get();
   LOG(info) << "Starting ratio from " << fTrackStart[index] << "(mm) " << tbFinal << "(TB)";

   // Get the track index that has the higher charge. This should be the numberator in the division
   int trackInd = fSumQ[0]->GetBinContent(tbIni + 1) > fSumQ[1]->GetBinContent(tbIni + 1) ? 0 : 1;

   fRatioQ->Divide(fSumQ[trackInd].get(), fSumQ[(trackInd + 1) % 2].get());
   fRatioFit->Divide(fSumFit[trackInd].get(), fSumFit[(trackInd + 1) % 2].get());

   // Fill the proxy function
   for (int i = 0; i < fSumFit[0]->GetNbinsX(); ++i) {
      auto de1 = fSumFit[0]->GetBinContent(i);
      auto de2 = fSumFit[1]->GetBinContent(i);
      if (de1 + de2 == 0)
         continue;
      auto proxy = std::abs(de1 - de2) / (de1 + de2);
      fProxy->SetBinContent(i, proxy);
      fZHist->SetBinContent(i, GetZ(85, proxy));
   }

   // Get the average of the ratio
   double ratioAvg = 0, proxyAvg = 0;

   for (int i = tbIni; i < tbFinal; ++i) {
      ratioAvg += fRatioFit->GetBinContent(i + 1);
      proxyAvg += fProxy->GetBinContent(i + 1);
   }
   ratioAvg /= fTBtoAvg.Get();
   proxyAvg /= fTBtoAvg.Get();

   LOG(info) << "Average ratio: " << ratioAvg << " Z avg: " << GetZ(85, proxyAvg) << "/" << 85 - GetZ(85, proxyAvg);

   // Set range of functions
   fRatioFunc->SetRange(tbIni, tbFinal);
   fZFunc->SetRange(tbIni, tbFinal);
   fProxyFunc->SetRange(tbIni, tbFinal);

   // Set value of functions
   fRatioFunc->SetParameter(0, ratioAvg);

   fZFunc->SetParameter(0, GetZ(85, proxyAvg));
   fProxyFunc->SetParameter(0, proxyAvg);
}

void AtTabEnergyLoss::FillSums(float threshold)
{
   if (fRawEvent.Get() == nullptr)
      return;
   for (int i = 0; i < 2; ++i) {
      fFirstHit[i] = nullptr;
      fTrackStart[i] = 0;

      // Fill fSumQ
      E12014::FillChargeSum(fSumQ[i].get(), fFissionEvent->GetFragHits(i), *fRawEvent, threshold);

      // Fill fSumFit
      E12014::FillHitSum(*fSumFit[i], fFissionEvent->GetFragHits(i), threshold, 4500);

      for (auto &hit : fFissionEvent->GetFragHits(i)) {
         auto inhibitType = AtViewerManager::Instance()->GetMap()->IsInhibited(hit->GetPadNum());
         if (inhibitType != AtMap::InhibitType::kNone)
            continue;

         // Update the first hit (want highest TB)
         auto hitLocation = hit->GetPosition().Z() - fSigmaFromHit.Get() * hit->GetPositionSigma().Z();
         if (fFirstHit[i] == nullptr) {
            fFirstHit[i] = hit;
         } else if (hitLocation <
                    fFirstHit[i]->GetPosition().Z() - fSigmaFromHit.Get() * fFirstHit[i]->GetPositionSigma().Z()) {
            fFirstHit[i] = hit;
         }

         // Update hit location

         if (fTrackStart[i] < hitLocation) {
            LOG(debug) << "Setting start of " << i << " to " << hit->GetPosition() << " at " << hit->GetPadNum();
            fTrackStart[i] = hitLocation;
         }
      }
   }
}
void AtTabEnergyLoss::setdEdX()
{
   for (int i = 0; i < 2; ++i) {

      for (auto &hit : fFissionEvent->GetFragHits(i))
         if (hit->GetPosition().z() > 0 && hit->GetPosition().Z() > fFissionEvent->GetVertex().Z()) {
            dEdx[i]->Fill(getHitDistanceFromVertex(*hit), hit->GetCharge());
            dEdxZ[i]->Fill(getHitDistanceFromVertexAlongZ(*hit), hit->GetCharge());
         }

      for (int bin = 0; bin < dEdx[i]->GetNbinsX(); ++bin) {
         dEdx[i]->SetBinError(bin, TMath::Sqrt(dEdx[i]->GetBinContent(bin)));
         dEdxZ[i]->SetBinError(bin, TMath::Sqrt(dEdxZ[i]->GetBinContent(bin)));
      }
   }
}

double AtTabEnergyLoss::getHitDistanceFromVertex(const AtHit &hit)
{
   auto p = hit.GetPosition();
   auto position = XYZPoint(p.x(), p.y(), p.z());
   auto diff = position - fFissionEvent->GetVertex();
   return TMath::Sqrt(diff.Mag2());
}

double AtTabEnergyLoss::getHitDistanceFromVertexAlongZ(const AtHit &hit)
{
   auto p = hit.GetPosition();
   auto diff = p.z() - fFissionEvent->GetVertex().z();
   return diff;
}
