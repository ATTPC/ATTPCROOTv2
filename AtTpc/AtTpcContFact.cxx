/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtTpcContFact.h"

#include <FairContFact.h>
#include <TList.h>
#include <TString.h>
#include <string.h>

#include "AtTpcGeoPar.h"
#include "FairRuntimeDb.h"

class FairParSet;

ClassImp(AtTpcContFact)

   static AtTpcContFact gAtTpcContFact;

AtTpcContFact::AtTpcContFact() : FairContFact()
{
   /** Constructor (called when the library is loaded) */
   fName = "AtTpcContFact";
   fTitle = "Factory for parameter containers in libAtTpc";
   setAllContainers();
   FairRuntimeDb::instance()->addContFactory(this);
}

void AtTpcContFact::setAllContainers()
{
   /** Creates the Container objects with all accepted
       contexts and adds them to
       the list of containers for the AtTpc library.
   */

   FairContainer *p = new FairContainer("AtTpcGeoPar", "AtTpc Geometry Parameters", "TestDefaultContext");
   p->addContext("TestNonDefaultContext");

   containers->Add(p);
}

FairParSet *AtTpcContFact::createContainer(FairContainer *c)
{
   /** Calls the constructor of the corresponding parameter container.
       For an actual context, which is not an empty string and not
       the default context
       of this container, the name is concatinated with the context.
   */
   const char *name = c->GetName();
   FairParSet *p = NULL;
   if (strcmp(name, "AtTpcGeoPar") == 0) {
      p = new AtTpcGeoPar(c->getConcatName().Data(), c->GetTitle(), c->getContext());
   }
   return p;
}
