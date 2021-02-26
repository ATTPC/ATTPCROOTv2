#include "AtSiArrayContFact.h"

#include "AtSiArrayGeoPar.h"

#include "FairRuntimeDb.h"

#include <iostream>

ClassImp(AtSiArrayContFact)

   static AtSiArrayContFact gAtSiArrayContFact;

AtSiArrayContFact::AtSiArrayContFact() : FairContFact()
{
   /** Constructor (called when the library is loaded) */
   fName = "AtSiArrayContFact";
   fTitle = "Factory for parameter containers in libAtSiArray";
   setAllContainers();
   FairRuntimeDb::instance()->addContFactory(this);
}

void AtSiArrayContFact::setAllContainers()
{
   /** Creates the Container objects with all accepted
       contexts and adds them to
       the list of containers for the AtTpc library.
   */

   FairContainer *p = new FairContainer("AtSiArrayGeoPar", "AtSiArray Geometry Parameters", "TestDefaultContext");
   p->addContext("TestNonDefaultContext");

   containers->Add(p);
}

FairParSet *AtSiArrayContFact::createContainer(FairContainer *c)
{
   /** Calls the constructor of the corresponding parameter container.
       For an actual context, which is not an empty string and not
       the default context
       of this container, the name is concatinated with the context.
   */
   const char *name = c->GetName();
   FairParSet *p = NULL;
   if (strcmp(name, "AtSiArrayGeoPar") == 0) {
      p = new AtSiArrayGeoPar(c->getConcatName().Data(), c->GetTitle(), c->getContext());
   }
   return p;
}
