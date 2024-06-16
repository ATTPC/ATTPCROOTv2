/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef SEGACONTFACT_H
#define SEGACONTFACT_H

#include <FairContFact.h>

#include <Rtypes.h> // for THashConsistencyHolder, ClassDef
class FairParSet;
class TBuffer;
class TClass;
class TMemberInspector;

class AtSeGAContFact : public FairContFact {
private:
   void setAllContainers();

public:
   AtSeGAContFact();
   ~AtSeGAContFact() {}
   FairParSet *createContainer(FairContainer *) override;
   ClassDefOverride(AtSeGAContFact, 1) // Factory for all AtSeGA parameter containers
};

#endif
