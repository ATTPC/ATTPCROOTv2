/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtApolloContFact.h"

#include <FairContFact.h>
#include <TString.h>
#include <stddef.h>

#include "FairRuntimeDb.h"

class FairParSet;

ClassImp(AtApolloContFact)

   static AtApolloContFact gAtApolloContFact;

AtApolloContFact::AtApolloContFact() : FairContFact()
{
   /** Constructor (called when the library is loaded) */
   fName = "AtApolloContFact";
   fTitle = "Factory for parameter containers in libAtApollo";
   setAllContainers();
   FairRuntimeDb::instance()->addContFactory(this);
}

void AtApolloContFact::setAllContainers()
{
   /** Creates the Container objects with all accepted
       contexts and adds them to
       the list of containers for the AtApollo library.
   */

   // FairContainer* p= new FairContainer("AtApolloGeoPar",
   //                                     "AtApollo Geometry Parameters",
   //                                       "TestDefaultContext");
   // p->addContext("TestNonDefaultContext");

   // containers->Add(p);
}

FairParSet *AtApolloContFact::createContainer(FairContainer *c)
{
   /** Calls the constructor of the corresponding parameter container.
       For an actual context, which is not an empty string and not
       the default context
       of this container, the name is concatinated with the context.
   */
   const char *name = c->GetName();
   FairParSet *p = NULL;
   // if (strcmp(name,"AtApolloGeoPar")==0) {
   //   p=new AtApolloGeoPar(c->getConcatName().Data(),
   //                           c->GetTitle(),c->getContext());
   // }
   return p;
}
