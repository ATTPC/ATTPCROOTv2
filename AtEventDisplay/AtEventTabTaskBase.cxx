#include "AtEventTabTaskBase.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtEventManagerNew.h" // for AtEventManager

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager

using namespace std;

ClassImp(AtEventTabTaskBase);

AtEventTabTaskBase::AtEventTabTaskBase(std::unique_ptr<AtEventTab> eventTab)
   : fTaskNumber(0), fEventTab(std::move(eventTab))
{
}

   AtEventTabTaskBase::~AtEventTabTaskBase() = default;

   void AtEventTabTaskBase::Exec(Option_t * option)
   {
      Reset();
   }

   void AtEventTabTaskBase::Reset()
   {
      fEventTab->Reset();
   }

   void AtEventTabTaskBase::MakeTab()
   {
      fEventTab->MakeTab();
   }

   void AtEventTabTaskBase::DrawPad(Int_t PadNum)
   {
      fEventTab->DrawPad(PadNum);
   }