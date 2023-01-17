#include "AtDataSubject.h"

#include "AtDataObserver.h"

using namespace DataHandling;

void Subject::Notify()
{
   for (auto obs : fObservers)
      obs->Update(this);
}
void Subject::Attach(Observer *observer)
{
   fObservers.insert(observer);
   observer->AttachToSubject(this);
}
