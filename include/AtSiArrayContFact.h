#ifndef ATSIARRAYCONTFACT_H
#define ATSIARRAYCONTFACT_H

#include "FairContFact.h"

class FairContainer;

class AtSiArrayContFact : public FairContFact
{
  private:
    void setAllContainers();
  public:
    AtSiArrayContFact();
    ~AtSiArrayContFact() {}
    FairParSet* createContainer(FairContainer*);
    ClassDef( AtSiArrayContFact,0) // Factory for all AtTpc parameter containers
};

#endif
