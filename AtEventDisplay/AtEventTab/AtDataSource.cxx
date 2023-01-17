#include "AtDataSource.h"

#include "AtTabInfo.h"
void SubjectBase::Notify()
{
   for (auto obs : fObservers)
      obs->Update(this);
}
void SubjectBase::Attach(AtTabInfo *observer)
{
   fObservers.insert(observer);
   observer->AttachToSubject(this);
}
