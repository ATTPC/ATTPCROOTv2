#include "AtSidebarAddon.h"

#include <TString.h>

ClassImp(AtSidebarAddon);

void AtSidebarAddon::AddIntBox(std::string title, std::string function, int min, int max)
{
   auto bFrame = new TGHorizontalFrame(this);
   auto bLabel = new TGLabel(bFrame, (TString)title+": ");
   auto bEntry = new TGNumberEntry(bFrame, 0., 6, -1, TGNumberFormat::kNESInteger, TGNumberFormat::kNEANonNegative,
                                   TGNumberFormat::kNELLimitMinMax, min, max);

   bEntry->Connect("ValueSet(Long_t)", ClassName(), this, (TString)function);
   bFrame->AddFrame(bLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   bFrame->AddFrame(bEntry);

   if (fNumbers.find(title) != fNumbers.end())
      std::cout << "Number titled " << title << "already exists in this addon!" << std::endl;
   else
      fNumbers.insert({title, bEntry});

   this->AddFrame(bFrame);
}
