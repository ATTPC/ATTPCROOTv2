#ifndef ATDATAOBSERVER_H
#define ATDATAOBSERVER_H

#include <set>

namespace DataHandling {
class Subject;
/**
 * This is the base class for any object that may observe data from a Subject and reacto on an update.
 *
 * It can register itself with any Subject and of that subject changes this class will be informed.
 * @ingroup DataHandling
 */
class Observer {
private:
   std::set<Subject *> fSubjects;

public:
   virtual ~Observer();
   virtual void Update(Subject *) = 0;
   void AttachToSubject(Subject *subject) { fSubjects.insert(subject); }
};
} // namespace DataHandling
#endif
