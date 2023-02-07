#ifndef ATVIEWERMANAGERSUBJECT_H
#define ATVIEWERMANAGERSUBJECT_H

#include "AtDataSubject.h"

class TString;

namespace DataHandling {

/**
 * @brief Subject for the entry number in the FairRoot tree.
 *
 * Is responsible for updating the run (ie calling FairRunAna::Instance()->Run()) before notifying
 * observers.
 * @ingroup DataHandling
 */
class AtTreeEntry : public AtSubject {
protected:
   long fEntry;

public:
   AtTreeEntry(long data) { fEntry = data; }
   long Get() const { return fEntry; }
   void Set(long entry);
};

/**
 * @brief Subject for the pad currently selected.
 *
 * @ingroup DataHandling
 */
using AtPadNum = AtSimpleType<int>;

/**
 * @brief Subject for the branch in the FairRoot tree.
 * @ingroup DataHandling
 */
class AtBranch : public AtSubject {
private:
   int fBranchId{-1};
   int fOldBranchId{-1};

public:
   /// Will notify on change
   void SetBranchName(const TString &name);
   /// Will notify on change
   void SetBranchId(const int id);

   TString GetBranchName() const;
   int GetBranchId() const { return fBranchId; }
   TString GetOldBranchName() const;
   int GetOldBranchId() const { return fOldBranchId; }
};
} // namespace DataHandling
#endif
