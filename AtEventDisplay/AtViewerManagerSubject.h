#ifndef ATVIEWERMANAGERSUBJECT_H
#define ATVIEWERMANAGERSUBJECT_H

#include "AtDataSubject.h"

class TString;

namespace DataHandling {

/**
 * @brief Subject for the entry number in the FairRoot tree.
 *
 * @ingroup DataHandling
 */
using AtTreeEntry = Simple<long>;

/**
 * @brief Subject for the pad currently selected.
 *
 * @ingroup DataHandling
 */
using AtPadNum = Simple<int>;

/**
 * @brief Subject for the branch in the FairRoot tree.
 * @ingroup DataHandling
 */
class AtBranch : public Subject {
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
