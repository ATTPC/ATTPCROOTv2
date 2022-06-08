#include "AtGeArrayContFact.h"

#include "AtGeArrayGeoPar.h"

#include <FairContFact.h>
#include <FairRuntimeDb.h>

#include <TList.h>
#include <TString.h>

#include <cstring>

class FairParSet;

ClassImp(AtGeArrayContFact)

   static AtGeArrayContFact gAtGeArrayContFact;

AtGeArrayContFact::AtGeArrayContFact() : FairContFact()
{
   /** Constructor (called when the library is loaded) */
   fName = "AtGeArrayContFact";
   fTitle = "Factory for parameter containers in libAtGeArray";
   setAllContainers();
   FairRuntimeDb::instance()->addContFactory(this);
}

void AtGeArrayContFact::setAllContainers()
{
   /** Creates the Container objects with all accepted
       contexts and adds them to
       the list of containers for the AtTpc library.
   */
   // NOLINTNEXTLINE (I think FairRoot owns this memory)
   auto *p = new FairContainer("AtGeArrayGeoPar", "AtGeArray Geometry Parameters", "TestDefaultContext");
   p->addContext("TestNonDefaultContext");

   containers->Add(p);
}

FairParSet *AtGeArrayContFact::createContainer(FairContainer *c)
{
   /** Calls the constructor of the corresponding parameter container.
       For an actual context, which is not an empty string and not
       the default context
       of this container, the name is concatinated with the context.
   */
   const char *name = c->GetName();
   FairParSet *p = nullptr;
   if (strcmp(name, "AtGeArrayGeoPar") == 0) {
      p = new AtGeArrayGeoPar(c->getConcatName().Data(), c->GetTitle(), c->getContext());
   }
   return p;
}
