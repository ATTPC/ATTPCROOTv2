#ifndef ATDATAOBSERVER_H
#define ATDATAOBSERVER_H

namespace DataHandling {
class AtSubject;
/**
 * This is the base class for any object that may observe data from a Subject and reacto on an update.
 *
 * It can register itself with any Subject and of that subject changes this class will be informed.
 * @ingroup DataHandling
 */
class AtObserver {
public:
   virtual ~AtObserver() = default;
   virtual void Update(AtSubject *) = 0;
};
} // namespace DataHandling
#endif
