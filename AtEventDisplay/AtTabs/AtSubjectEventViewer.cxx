#include "AtSubjectEventViewer.h"

#include <iostream>
namespace DataHandling {

void AtEntryNumber::Set(long eventNum)
{
   if (fEntryNum == eventNum)
      return;
   fEntryNum = eventNum;
   Notify();
}

void AtBranchName::SetBranchName(std::string name)
{
   if (fBranchName.compare(name) == 0)
      return;
   std::cout << "Setting branch name to " << name << std::endl;
   fOldBranchName = fBranchName;
   fBranchName = name;
   Notify();
}

} // namespace DataHandling
