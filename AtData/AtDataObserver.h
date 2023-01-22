#ifndef ATDATAOBSERVER_H
#define ATDATAOBSERVER_H

namespace DataHandling {
class Subject;
/**
 * This is the base class for any object that may observe data from a Subject and reacto on an update.
 *
 * It can register itself with any Subject and of that subject changes this class will be informed.
 * @ingroup DataHandling
 */
class Observer {
public:
   virtual ~Observer() = default;
   virtual void Update(Subject *) = 0;
};
} // namespace DataHandling
#endif
