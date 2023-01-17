#ifndef ATDATASUBJECT_H
#define ATDATASUBJECT_H

#include <set>
#include <string>

/**
 @defgroup DataHandling Data Handling
 *
 * Classes for storing information used by other classes that may be updated at arbitrary time. All classes in this
 group derive either from Subject or Observer. Subject classes store data and if that data is updated will notify every
 Observer attached to them of the change. The observer can then decide how to respond.

*/
namespace DataHandling {
class Observer;
/**
 * This is the base class for any object that may contain data to send to Observer on an update.
 *
 * Any Observer may regiter itself with these subjects, and they will be notified when the state of
 * this object changes, then they can retrieve the updated information and act on it.
 *
 */
class Subject {
private:
   std::set<Observer *> fObservers;

protected:
   Subject() = default;

public:
   virtual ~Subject() = default;

   /// Attach an observer to get notified when this subject changes
   void Attach(Observer *observer);
   /// Detach an observer to stop getting notified when this subject changes
   void Detach(Observer *observer) { fObservers.erase(observer); }
   /// Notify all attached subjects that something changed
   void Notify();
};

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
