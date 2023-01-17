#ifndef ATDATAOBSERVER_H
#define ATDATAOBSERVER_H

#include <set>

namespace DataHandling {
class Subject;

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
