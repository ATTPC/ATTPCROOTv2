#include "AtSidebarPSAIterDeconv.h"

#include "AtPSAIterDeconv.h"

void AtSidebarPSAIterDeconv::FillFrame()
{
   fThresholdFrame = new TGHorizontalFrame(this);
   fThresholdLabel = new TGLabel(fThresholdFrame, "Threshold: ");
   fThresholdEntry = new TGNumberEntry(fThresholdFrame, 0., 6, -1, TGNumberFormat::kNESInteger,
                                          TGNumberFormat::kNEANonNegative, TGNumberFormat::kNELLimitMinMax, -1,
                                          4000);

   fThresholdEntry->Connect("ValueSet(Int_t)", "AtPSAIterDeconv", this, "SetThreshold()");
   (fThresholdEntry->GetNumberEntry())->Connect("ReturnPressed()", "AtPSAIterDeconv", this, "SetThreshold()");
   fThresholdFrame->AddFrame(fThresholdLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   fThresholdFrame->AddFrame(fThresholdEntry);

   fOrderFrame = new TGHorizontalFrame(this);
   fOrderLabel = new TGLabel(fOrderFrame, "Filter Order: ");
   fOrderEntry = new TGNumberEntry(fOrderFrame, 0., 6, -1, TGNumberFormat::kNESInteger,
                                          TGNumberFormat::kNEANonNegative, TGNumberFormat::kNELLimitMinMax, -1, 16
                                          );

   fOrderEntry->Connect("ValueSet(Int_t)", "AtPSAIterDeconv", this, "SetOrder()");
   fOrderFrame->AddFrame(fOrderLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   fOrderFrame->AddFrame(fOrderEntry);

   fCutoffFrame = new TGHorizontalFrame(this);
   fCutoffLabel = new TGLabel(fCutoffFrame, "Filter Cutoff: ");
   fCutoffEntry = new TGNumberEntry(fCutoffFrame, 0., 6, -1, TGNumberFormat::kNESInteger,
                                          TGNumberFormat::kNEANonNegative, TGNumberFormat::kNELLimitMinMax, -1,
                                          512);

   fCutoffEntry->Connect("ValueSet(Int_t)", "AtPSAIterDeconv", this, "SetCutoff()");
   fCutoffFrame->AddFrame(fCutoffLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   fCutoffFrame->AddFrame(fCutoffEntry);

   fIterationsFrame = new TGHorizontalFrame(this);
   fIterationsLabel = new TGLabel(fIterationsFrame, "Iterations: ");
   fIterationsEntry = new TGNumberEntry(fIterationsFrame, 0., 6, -1, TGNumberFormat::kNESInteger,
                                          TGNumberFormat::kNEANonNegative, TGNumberFormat::kNELLimitMinMax, -1,
                                          10);

   fIterationsEntry->Connect("ValueSet(Int_t)", "AtPSAIterDeconv", this, "SetIterations()");
   fIterationsFrame->AddFrame(fIterationsLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   fIterationsFrame->AddFrame(fIterationsEntry);

    this->AddFrame(fThresholdFrame);
    this->AddFrame(fOrderFrame);
    this->AddFrame(fCutoffFrame);
    this->AddFrame(fIterationsFrame);

    fThresholdEntry->GetNumberEntry()->SetIntNumber(fPSA->GetThreshold());
    fOrderEntry->GetNumberEntry()->SetIntNumber(fPSA->GetFilterOrder());
    fCutoffEntry->GetNumberEntry()->SetIntNumber(fPSA->GetCutoffFreq());
    fIterationsEntry->GetNumberEntry()->SetIntNumber(fPSA->GetIterations());
}

