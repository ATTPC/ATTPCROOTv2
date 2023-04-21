#include "AtTabFF.h"

#include "AtE12014.h"
#include "AtViewerManager.h"

#include <TCanvas.h>
#include <TString.h>

AtTabFF::AtTabFF(DataHandling::AtBranch &fissionBranch)
   : AtTabCanvas("FF", 2, 1), fEvent(AtViewerManager::Instance()->GetEventBranch()), fFissionEvent(fissionBranch),
     fEntry(AtViewerManager::Instance()->GetCurrentEntry())
{
   std::array<Color_t, 2> colors = {9, 31};
   for (int i = 0; i < 2; ++i) {
      fSimdQdZ[i] = std::make_unique<TH1F>(TString::Format("sim%d", i), TString::Format("Sim FF %d", i), 512, 0, 512);
      fExpdQdZ[i] = std::make_unique<TH1F>(TString::Format("exp%d", i), TString::Format("Exp FF %d", i), 512, 0, 512);

      fSimdQdZ[i]->SetDirectory(nullptr);
      fExpdQdZ[i]->SetDirectory(nullptr);
      fSimdQdZ[i]->SetLineColor(kRed);
      fExpdQdZ[i]->SetLineColor(kBlue);
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
   if (sub == &fEntry) {
      UpdateExpEvent();
      UpdateSimEvent();
      DrawCanvas();
   } else if (sub == &AtViewerManager::Instance()->GetEventBranch()) {
      UpdateSimEvent();
      DrawCanvas();
   }
}

void AtTabFF::UpdateSimEvent()
{
   if (fEvent.Get() == nullptr)
      return;

   for (int i = 0; i < 2; ++i) {
      E12014::FillSimHitSum(*fSimdQdZ[i], ContainerManip::GetPointerVector(fEvent->GetHits()), fCurrPads[i], 15);
   }
}
void AtTabFF::UpdateExpEvent()
{
   for (int i = 0; i < 2; ++i) {
      fCurrPads[i] = E12014::FillHitSum(*fExpdQdZ[i], fFissionEvent->GetFragHits(i), 15);
   }
}

void AtTabFF::DrawCanvas()
{
   for (int i = 0; i < 2; ++i) {
      fCanvas->cd(i + 1);
      fSimdQdZ[i]->Draw("hist");
      fExpdQdZ[i]->Draw("same");
   }
   UpdateCanvas();
}
ClassImp(AtTabFF);
