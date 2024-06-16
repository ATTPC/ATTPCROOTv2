/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef PXCTCONTFACT_H
#define PXCTCONTFACT_H

#include <FairContFact.h>

#include <Rtypes.h> // for THashConsistencyHolder, ClassDef
class FairParSet;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPxCTContFact : public FairContFact {
private:
   void setAllContainers();

public:
   AtPxCTContFact();
   ~AtPxCTContFact() {}
   FairParSet *createContainer(FairContainer *) override;
   ClassDefOverride(AtPxCTContFact, 1) // Factory for all AtPxCT parameter containers
};

#endif
