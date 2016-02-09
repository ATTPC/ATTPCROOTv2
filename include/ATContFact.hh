#ifndef STCONTFACT_H
#define STCONTFACT_H

#include "FairContFact.h"

class FairContainer;

class ATContFact : public FairContFact
{
  public:
    ATContFact();
    ~ATContFact();

    FairParSet* createContainer(FairContainer*);

  private:
    void setAllContainers();

  ClassDef(ATContFact, 1) // Factory for all SPiRIT parameter containers
};

#endif
