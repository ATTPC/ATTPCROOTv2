#include "ATContFact.hh"
//#include "ATGeoPar.hh"
#include "ATDigiPar.hh"
#include "ATTriggerPar.hh"

#include "FairRuntimeDb.h"

#include <iostream>

ClassImp(ATContFact)

static ATContFact gATContFact;

ATContFact::ATContFact()
:FairContFact()
{
  /** Constructor (called when the library is loaded) */
  fName = "ATContFact";
  fTitle = "Factory for parameter containers in libATTPC";
  setAllContainers();
  FairRuntimeDb::instance() -> addContFactory(this);
}

ATContFact::~ATContFact()
{
}

void ATContFact::setAllContainers()
{
  /** Creates the Container objects with all accepted
      contexts and adds them to
      the list of containers for the SPiRIT library.
  */

  /*FairContainer* p = new FairContainer("STGeoPar",
                                       "SPiRIT Geometry Parameters",
                                       "TestDefaultContext");

  containers -> Add(p);*/

  FairContainer* p = new FairContainer("ATDigiPar",
                        "ATTPC Parameter Container",
                        "");

  containers -> Add(p);

  FairContainer* pp = new FairContainer("ATTriggerPar",
                          "ATTPC Parameter Container",
                          "");

  containers -> Add(pp);
}

FairParSet* ATContFact::createContainer(FairContainer* c)
{
  /** Calls the constructor of the corresponding parameter container.
      For an actual context, which is not an empty string and not
      the default context
      of this container, the name is concatinated with the context.
  */
  const char* name = c -> GetName();
  FairParSet* p = NULL;

  /*if (strcmp(name, "STGeoPar") == 0) {
    p = new STGeoPar(c->getConcatName().Data(),
                     c->GetTitle(),c->getContext());
  }*/

  if (strcmp(name, "ATDigiPar") == 0) {
    p = new ATDigiPar(c -> getConcatName().Data(),
                      c -> GetTitle(), c -> getContext());
  }
  if (strcmp(name, "ATTriggerPar") == 0) {
    p = new ATTriggerPar(c -> getConcatName().Data(),
                      c -> GetTitle(), c -> getContext());
  }
  return p;
}
