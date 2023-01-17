#include "AtDataObserver.h"

#include "AtDataSubject.h"
using namespace DataHandling;

Observer::~Observer()
{
   for (auto sub : fSubjects)
      sub->Detach(this);
}
