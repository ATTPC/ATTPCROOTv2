#ifndef ATVIEWERMANAGERSUBJECT_H
#define ATVIEWERMANAGERSUBJECT_H

#include "AtDataSubject.h"

#include <string>

namespace DataHandling {

/**
 * Subject for storing the entry number in the tree
 */
class AtEntryNumber : public Subject {
private:
   long fEntryNum;

public:
   AtEntryNumber(long eventNum) : fEntryNum(eventNum) {}

   long Get() const { return fEntryNum; }

   /// Will notify on change
   void Set(long eventNum);
};

/**
 * Branch name subject
 */
class AtBranchName : public Subject {
private:
   std::string fBranchName{""};
   std::string fOldBranchName{""};

public:
   AtBranchName() {}

   /// Will notify on change
   void SetBranchName(std::string name);

   std::string GetBranchName() const { return fBranchName; }
   std::string GetOldBranchName() const { return fOldBranchName; }
};
} // namespace DataHandling
#endif
