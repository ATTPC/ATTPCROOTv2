/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef APOLLOCONTFACT_H
#define APOLLOCONTFACT_H

#include "FairContFact.h"

class FairContainer;

class AtApolloContFact : public FairContFact
{
  private:
    void setAllContainers();
  public:
    AtApolloContFact();
    ~AtApolloContFact() {}
    FairParSet* createContainer(FairContainer*);
    ClassDef(AtApolloContFact,0) // Factory for all AtApollo parameter containers
};

#endif
