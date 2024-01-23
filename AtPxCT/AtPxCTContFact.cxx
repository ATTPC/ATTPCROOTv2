/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtPxCTContFact.h"

#include <FairRuntimeDb.h>

#include <TString.h> // for TString

ClassImp(AtPxCTContFact);

static AtPxCTContFact gAtPxCTContFact;

AtPxCTContFact::AtPxCTContFact() : FairContFact()
{
   /** Constructor (called when the library is loaded) */
   fName = "AtPxCTContFact";
   fTitle = "Factory for parameter containers in libAtPxCT";
   setAllContainers();
   FairRuntimeDb::instance()->addContFactory(this);
}

void AtPxCTContFact::setAllContainers()
{
   /** Creates the Container objects with all accepted
       contexts and adds them to
       the list of containers for the AtPxCT library.
   */

   // FairContainer* p= new FairContainer("AtPxCTGeoPar",
   //                                     "AtPxCT Geometry Parameters",
   //                                       "TestDefaultContext");
   // p->addContext("TestNonDefaultContext");

   // containers->Add(p);
}

FairParSet *AtPxCTContFact::createContainer(FairContainer *c)
{
   return nullptr;
}
