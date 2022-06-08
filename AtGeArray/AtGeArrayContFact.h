#ifndef AtGEARRAYCONTFACT_H
#define AtGEARRAYCONTFACT_H

#include <FairContFact.h>

#include <Rtypes.h>

class FairParSet;
class TBuffer;
class TClass;
class TMemberInspector;

class AtGeArrayContFact : public FairContFact {
private:
   void setAllContainers();

public:
   AtGeArrayContFact();
   ~AtGeArrayContFact() {}
   FairParSet *createContainer(FairContainer *);
   ClassDef(AtGeArrayContFact, 0) // Factory for all AtTpc parameter containers
};

#endif
