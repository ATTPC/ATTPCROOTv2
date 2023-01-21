#include "AtSidebarAddon.h"

#include <FairLogger.h>

#include <Rtypes.h>        // for TGenericClassInfo
#include <TGFrame.h>       // for TGHorizontalFrame
#include <TGLabel.h>       // for TGLabel
#include <TGLayout.h>      // for TGLayoutHints, kLHintsCenterY, kLHintsLeft
#include <TGNumberEntry.h> // for TGNumberEntry, TGNumberFormat, TGNumberFo...
#include <TString.h>

#include <utility> // for pair

ClassImp(AtSidebarAddon);

void AtSidebarAddon::AddIntBox(std::string label, std::string function, int min, int max)
{
   auto bFrame = new TGHorizontalFrame(this);
   auto bLabel = new TGLabel(bFrame, (TString)label + ": ");

   auto bEntry = new TGNumberEntry(bFrame, 0., 6, -1, TGNumberFormat::kNESInteger, TGNumberFormat::kNEANonNegative,
                                   TGNumberFormat::kNELLimitMinMax, min, max);

   bEntry->Connect("ValueSet(Long_t)", ClassName(), this, (TString)function);
   bFrame->AddFrame(bLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   bFrame->AddFrame(bEntry);

   if (fNumbers.find(label) != fNumbers.end())
      LOG(error) << "Number labeled " << label << "already exists in this addon!";
   else
      fNumbers.insert({label, bEntry});

   this->AddFrame(bFrame);
}

Long_t AtSidebarAddon::GetIntNumber(std::string label)
{
   if (fNumbers.find(label) == fNumbers.end()) {
      LOG(error) << label << " not defined!";
      return 0;
   } else
      return fNumbers.find(label)->second->GetNumberEntry()->GetIntNumber();
}

void AtSidebarAddon::SetIntNumber(std::string label, Long_t value)
{
   if (fNumbers.find(label) == fNumbers.end())
      LOG(error) << label << " not defined!";
   else
      fNumbers.find(label)->second->GetNumberEntry()->SetIntNumber(value);
}
