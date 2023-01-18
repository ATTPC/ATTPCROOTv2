#ifndef ATEVENTVIEWERSUBJECT_H
#define ATEVENTVIEWERSUBJECT_H

#include "AtDataSubject.h"

#include <string>

namespace DataHandling {

/**
 * This is the base class used to pass around
 *
 * Any AtTabInfo may regiter itself with these subjects, and they will be notified when the state of
 * this object changes, then can then retrieve the updated information and update themselves.
 *
 */
class BranchName : public Subject {
protected:
   std::string fBranchType;
   std::string fBranchName;
   std::string fOldBranchName;

public:
   BranchName(std::string type, std::string name, std::string oldName)
      : fBranchType(type), fBranchName(name), fOldBranchName(oldName)
   {
   }
   void SetBranchName(std::string name)
   {
      fOldBranchName = fBranchName;
      fBranchName = name;
   }
   std::string GetBranchType() { return fBranchType; }
   std::string GetBranchName() { return fBranchName; }
   std::string GetOldBranchName() { return fOldBranchName; }
};
} // namespace DataHandling
#endif
