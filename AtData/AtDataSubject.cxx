#include "AtDataSubject.h"

#include "AtDataObserver.h"

using namespace DataHandling;

void AtSubject::Notify()
{
   for (auto obs : fObservers)
      obs->Update(this);
}
void AtSubject::Attach(AtObserver *observer)
{
   fObservers.insert(observer);
}
