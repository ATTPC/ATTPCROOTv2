#ifndef STCONTFACT_H
#define STCONTFACT_H

#include <FairContFact.h>

#include <Rtypes.h>

class FairParSet;
class TBuffer;
class TClass;
class TMemberInspector;

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
