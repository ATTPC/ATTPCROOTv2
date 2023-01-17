#ifndef ATDATASOURCE_H
#define ATDATASOURCE_H

#include <set>
#include <string>

class AtTabInfo;

/**
 * This is the base class for any object that may contain data to send to AtTabInfos on an update.
 *
 * Any AtTabInfo may regiter itself with these subjects, and they will be notified when the state of
 * this object changes, then can then retrieve the updated information and update themselves.
 *
 */
class SubjectBase {
private:
   std::set<AtTabInfo *> fObservers;

protected:
   SubjectBase() = default;

public:
   virtual ~SubjectBase() = default;

   /// Attach an observer to get notified when this subject changes
   void Attach(AtTabInfo *observer);
   /// Detach an observer to stop getting notified when this subject changes
   void Detach(AtTabInfo *observer) { fObservers.erase(observer); }
   /// Notify all attached subjects that something changed
   void Notify();
};

class BranchName : public SubjectBase {
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

#endif
