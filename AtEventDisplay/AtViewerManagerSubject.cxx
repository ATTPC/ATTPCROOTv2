#include "AtViewerManagerSubject.h"

#include <FairRootManager.h>

#include <TString.h>
namespace DataHandling {

void AtTreeEntry::Set(long eventNum)
{
   if (fEntry == eventNum)
      return;
   fEntry = eventNum;
   Notify();
}

void AtBranch::SetBranchName(const TString &name)
{
   SetBranchId(FairRootManager::Instance()->GetBranchId(name));
}

void AtBranch::SetBranchId(const int id)
{
   if (fBranchId == id)
      return;

   fOldBranchId = fBranchId;
   fBranchId = id;
   Notify();
}

TString AtBranch::GetBranchName() const
{
   return FairRootManager::Instance()->GetBranchName(fBranchId);
}

TString AtBranch::GetOldBranchName() const
{
   return FairRootManager::Instance()->GetBranchName(fOldBranchId);
}

} // namespace DataHandling
