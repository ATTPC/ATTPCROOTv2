#include "AtTabFF.h"

#include "AtContainerManip.h" // for GetPointerVector
#include "AtE12014.h"
#include "AtMCResult.h"
#include "AtViewerManager.h"
#include "AtViewerManagerSubject.h" // for AtBranch, AtTreeEntry

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager

#include <TCanvas.h>
#include <TClonesArray.h> // for TClonesArray
#include <TH1.h>          // for TH1F
#include <THStack.h>      // for THStack
#include <TObject.h>      // for TObject
#include <TString.h>

#include <ostream> // for endl
namespace DataHandling {
class AtSubject;
}

AtTabFF::AtTabFF(DataHandling::AtBranch &fissionBranch, bool plotADC)
   : AtTabCanvas("FF", 2, 2), fEvent(AtViewerManager::Instance()->GetEventBranch()),
     fRawEvent(AtViewerManager::Instance()->GetRawEventBranch()), fFissionEvent(fissionBranch),
     fEntry(AtViewerManager::Instance()->GetCurrentEntry())
{
   fStacks[0] = std::make_unique<THStack>("hFF1", "FF 1");
   fStacks[2] = std::make_unique<THStack>("hFF2", "FF 2");
   fStacks[1] = std::make_unique<THStack>("hExp", "Experimental dE/dX curves");
   if (plotADC)
      fStacks[3] = std::make_unique<THStack>("hSim", "Experimental ADC Sum");
   else
      fStacks[3] = std::make_unique<THStack>("hSim", "Simulated dE/dX curves");

   std::array<Color_t, 2> colors = {9, 31};
   for (int i = 0; i < 2; ++i) {
      fSimdQdZ[i] = std::make_unique<TH1F>(TString::Format("sim%d", i), TString::Format("Sim FF %d", i), 512, 0, 512);
      fExpdQdZ[i] = std::make_unique<TH1F>(TString::Format("exp%d", i), TString::Format("Exp FF %d", i), 512, 0, 512);
      fExpADCSum[i] =
         std::make_unique<TH1F>(TString::Format("expADC%d", i), TString::Format("Exp FF ADC Sum %d", i), 512, 0, 512);
      ;
      fSimdQdZ[i]->SetDirectory(nullptr);
      fExpdQdZ[i]->SetDirectory(nullptr);
      fExpADCSum[i]->SetDirectory(nullptr);

      fSimdQdZ[i]->SetLineColor(kRed);
      fExpdQdZ[i]->SetLineColor(kBlue);
      fExpADCSum[i]->SetLineColor(kBlue);

      // FF i should be added to i*2 (they're the first coloumn
      fStacks[i * 2]->Add(fSimdQdZ[i].get());
      fStacks[i * 2]->Add(fExpdQdZ[i].get());

      fStacks[1]->Add(fExpdQdZ[i].get());
      if (plotADC)
         fStacks[3]->Add(fExpADCSum[i].get());
      else
         fStacks[3]->Add(fSimdQdZ[i].get());
   }

   fEntry.Attach(this);
   AtViewerManager::Instance()->GetEventBranch().Attach(this);
}

AtTabFF::~AtTabFF()
{
   fEntry.Detach(this);
   AtViewerManager::Instance()->GetEventBranch().Detach(this);
}

void AtTabFF::Update(DataHandling::AtSubject *sub)
{
   UpdateEvent();
   DrawCanvas();
}

std::vector<AtHit *> AtTabFF::GetFragmentHits(AtEvent *event)
{

   return {};
}

void AtTabFF::UpdateEvent()
{
   if (fEvent.Get() == nullptr)
      return;

   for (int i = 0; i < 2; ++i) {
      std::vector<double> exp;
      std::vector<double> sim;
      std::vector<double> adcSum;
      auto tbMin = E12014::fTBMin;
      E12014::fTBMin = 0;
      int goodHits = E12014::FillHitSums(exp, sim, fFissionEvent->GetFragHits(i),
                                         ContainerManip::GetPointerVector(fEvent->GetHits()), 0, E12014::fSatThreshold,
                                         nullptr, &adcSum, fRawEvent.Get());
      LOG(info) << "Good hits " << goodHits;
      /*
      using namespace SampleConsensus;
      AtSampleConsensus sac(Estimators::kRANSAC, AtPatterns::PatternType::kLine, RandomSample::SampleMethod::kUniform);
      auto patEvent = sac.Solve(fEvent.Get());

      E12014::FillSimSum(sim, ContainerManip::GetPointerVector(patEvent.GetTrackCand()[1].GetHitArray()));
      */
      E12014::fTBMin = tbMin;
      ContainerManip::SetHistFromData(*fExpdQdZ[i], exp);
      ContainerManip::SetHistFromData(*fSimdQdZ[i], sim);
      ContainerManip::SetHistFromData(*fExpADCSum[i], adcSum);
      auto fResultArray = dynamic_cast<TClonesArray *>(FairRootManager::Instance()->GetObject("AtMCResult"));
      if (fResultArray) {
         auto result = dynamic_cast<MCFitter::AtMCResult *>(fResultArray->At(0));
         fSimdQdZ[i]->Scale(result->fParameters["Amp"]);
         if (i == 0) {
            LOG(info) << "Z1: " << result->fParameters["Z0"] << " A1: " << result->fParameters["A0"];
            LOG(info) << "Z2: " << result->fParameters["Z1"] << " A2: " << result->fParameters["A1"];
            LOG(info) << "ObjQ: " << result->fParameters["ObjQ"] << " ObjPos: " << result->fParameters["ObjPos"]
                      << " Amp: " << result->fParameters["Amp"];
            bool inCut = result->fParameters["ObjQ"] < 15 && result->fParameters["Amp"] < 0.5;
            LOG(info) << std::endl << (inCut ? "In Cut" : "Out Cut") << std::endl;
         }
      }
   }
}

void AtTabFF::DrawCanvas()
{
   // Loop through each stack. If its an even
   for (int i = 0; i < 4; ++i) {
      fCanvas->cd(i + 1);
      // We are drawing a sim vs experiment
      if (i % 2 == 0) {
         fSimdQdZ[i / 2]->SetLineColor(kRed);
         fExpdQdZ[i / 2]->SetLineColor(kBlue);
         fStacks[i]->Draw("nostack;hist");
      } else if (i == 1) { // Drawing experimental stack
         // fExpdQdZ[0]->SetLineColor(9);
         // fExpdQdZ[1]->SetLineColor(31);
         fStacks[i]->Draw("nostack;hist");
      } else if (i == 3) { // Drawing simulation stack
         // fSimdQdZ[0]->SetLineColor(9);
         // fSimdQdZ[1]->SetLineColor(31);
         fStacks[i]->Draw("nostack;hist");
      }
   }
   UpdateCanvas();
}
ClassImp(AtTabFF);
