#ifndef STCONTFACT_H
#define STCONTFACT_H

#include "FairContFact.h"

class FairContainer;

class AtContFact : public FairContFact {
public:
   AtContFact();
   ~AtContFact();

   FairParSet *createContainer(FairContainer *);

private:
   void setAllContainers();

   ClassDef(AtContFact, 1) // Factory for all SPiRIT parameter containers
};

#endif
