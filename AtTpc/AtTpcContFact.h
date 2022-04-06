/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef NEWDETECTORCONTFACT_H
#define NEWDETECTORCONTFACT_H

#include <Rtypes.h>

#include "FairContFact.h"

class FairParSet;
class TBuffer;
class TClass;
class TMemberInspector;

class AtTpcContFact : public FairContFact {
private:
   void setAllContainers();

public:
   AtTpcContFact();
   ~AtTpcContFact() {}
   FairParSet *createContainer(FairContainer *);
   ClassDef(AtTpcContFact, 0) // Factory for all AtTpc parameter containers
};

#endif
