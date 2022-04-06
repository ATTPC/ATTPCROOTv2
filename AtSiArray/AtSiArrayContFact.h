#ifndef AtSIARRAYCONTFACT_H
#define AtSIARRAYCONTFACT_H

#include <Rtypes.h>

#include "FairContFact.h"

class FairParSet;
class TBuffer;
class TClass;
class TMemberInspector;

class AtSiArrayContFact : public FairContFact {
private:
   void setAllContainers();

public:
   AtSiArrayContFact();
   ~AtSiArrayContFact() {}
   FairParSet *createContainer(FairContainer *);
   ClassDef(AtSiArrayContFact, 0) // Factory for all AtTpc parameter containers
};

#endif
