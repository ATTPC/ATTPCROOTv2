/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtDeGAiContFact.h"

#include <FairRuntimeDb.h>

#include <TString.h> // for TString

ClassImp(AtDeGAiContFact);

static AtDeGAiContFact gAtDeGAiContFact;

AtDeGAiContFact::AtDeGAiContFact() : FairContFact()
{
   /** Constructor (called when the library is loaded) */
   fName = "AtDeGAiContFact";
   fTitle = "Factory for parameter containers in libAtDeGAi";
   setAllContainers();
   FairRuntimeDb::instance()->addContFactory(this);
}

void AtDeGAiContFact::setAllContainers()
{
   /** Creates the Container objects with all accepted
       contexts and adds them to
       the list of containers for the AtDeGAi library.
   */

   // FairContainer* p= new FairContainer("AtDeGAiGeoPar",
   //                                     "AtDeGAi Geometry Parameters",
   //                                       "TestDefaultContext");
   // p->addContext("TestNonDefaultContext");

   // containers->Add(p);
}

FairParSet *AtDeGAiContFact::createContainer(FairContainer *c)
{
   return nullptr;
}
