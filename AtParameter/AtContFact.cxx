#include "AtContFact.h"

#include <FairContFact.h>

#include <TList.h>
#include <TString.h>

#include <cstring>

//#include "AtGeoPar.h"
#include "AtDigiPar.h"
#include "AtTriggerPar.h"

#include <FairRuntimeDb.h>

class FairParSet;

ClassImp(AtContFact)

   static AtContFact gAtContFact;

AtContFact::AtContFact() : FairContFact()
{
   /** Constructor (called when the library is loaded) */
   fName = "AtContFact";
   fTitle = "Factory for parameter containers in libAtTPC";
   setAllContainers();
   FairRuntimeDb::instance()->addContFactory(this);
}

AtContFact::~AtContFact() = default;

void AtContFact::setAllContainers()
{
   /** Creates the Container objects with all accepted
       contexts and adds them to
       the list of containers for the SPiRIT library.
   */

   /*FairContainer* p = new FairContainer("STGeoPar",
                                        "SPiRIT Geometry Parameters",
                                        "TestDefaultContext");

   containers -> Add(p);*/

   auto *p = new FairContainer("AtDigiPar", "AtTPC Parameter Container", ""); // NOLINT

   containers->Add(p);

   auto *pp = new FairContainer("AtTriggerPar", "AtTPC Parameter Container", ""); // NOLINT

   containers->Add(pp);
}

FairParSet *AtContFact::createContainer(FairContainer *c)
{
   /** Calls the constructor of the corresponding parameter container.
       For an actual context, which is not an empty string and not
       the default context
       of this container, the name is concatinated with the context.
   */
   const char *name = c->GetName();
   FairParSet *p = nullptr;

   /*if (strcmp(name, "STGeoPar") == 0) {
     p = new STGeoPar(c->getConcatName().Data(),
                      c->GetTitle(),c->getContext());
   }*/

   if (strcmp(name, "AtDigiPar") == 0) {
      p = new AtDigiPar(c->getConcatName().Data(), c->GetTitle(), c->getContext());
   }
   if (strcmp(name, "AtTriggerPar") == 0) {
      p = new AtTriggerPar(c->getConcatName().Data(), c->GetTitle(), c->getContext());
   }
   return p;
}
