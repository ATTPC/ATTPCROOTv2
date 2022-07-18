/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtSeGAContFact.h"

#include <FairRuntimeDb.h>

#include <TString.h> // for TString

#include <iostream>
ClassImp(AtSeGAContFact)

   static AtSeGAContFact gAtSeGAContFact;

AtSeGAContFact::AtSeGAContFact() : FairContFact()
{
   /** Constructor (called when the library is loaded) */
   fName = "AtSeGAContFact";
   fTitle = "Factory for parameter containers in libAtSeGA";
   setAllContainers();
   FairRuntimeDb::instance()->addContFactory(this);
}

void AtSeGAContFact::setAllContainers()
{
   /** Creates the Container objects with all accepted
       contexts and adds them to
       the list of containers for the AtSeGA library.
   */

   // FairContainer* p= new FairContainer("AtSeGAGeoPar",
   //                                     "AtSeGA Geometry Parameters",
   //                                       "TestDefaultContext");
   // p->addContext("TestNonDefaultContext");

   // containers->Add(p);
}

FairParSet *AtSeGAContFact::createContainer(FairContainer *c)
{
   /** Calls the constructor of the corresponding parameter container.
       For an actual context, which is not an empty string and not
       the default context
       of this container, the name is concatinated with the context.
   */
   const char *name = c->GetName();
   FairParSet *p = nullptr;
   // if (strcmp(name,"AtSeGAGeoPar")==0) {
   //   p=new AtSeGAGeoPar(c->getConcatName().Data(),
   //                           c->GetTitle(),c->getContext());
   // }
   return p;
}
