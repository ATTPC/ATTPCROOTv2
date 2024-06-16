/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef DEGAICONTFACT_H
#define DEGAICONTFACT_H

#include <FairContFact.h>

#include <Rtypes.h> // for THashConsistencyHolder, ClassDef
class FairParSet;
class TBuffer;
class TClass;
class TMemberInspector;

class AtDeGAiContFact : public FairContFact {
private:
   void setAllContainers();

public:
   AtDeGAiContFact();
   ~AtDeGAiContFact() {}
   FairParSet *createContainer(FairContainer *) override;
   ClassDefOverride(AtDeGAiContFact, 1) // Factory for all AtDeGAi parameter containers
};

#endif
