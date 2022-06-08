/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef SEGACONTFACT_H
#define SEGACONTFACT_H

#include "FairContFact.h"

class FairContainer;

class AtSeGAContFact : public FairContFact {
private:
   void setAllContainers();

public:
   AtSeGAContFact();
   ~AtSeGAContFact() {}
   FairParSet *createContainer(FairContainer *);
   ClassDef(AtSeGAContFact, 0) // Factory for all AtSeGA parameter containers
};

#endif
