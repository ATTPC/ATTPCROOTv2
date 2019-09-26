/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

// -------------------------------------------------------------------------
// -----                    AtMagnet  file                               -----
// -----                Created 26/03/14  by M. Al-Turany              -----
// -------------------------------------------------------------------------


#ifndef MAGNET_H
#define MAGNET_H

#include "FairModule.h"                 // for FairModule
#include "Rtypes.h"                     // for AtMagnet::Class, Bool_t, etc
#include <string>                       // for string

class AtMagnet : public FairModule
{
  public:
    AtMagnet(const char* name, const char* Title="MY Magnet");
    AtMagnet();
    virtual ~AtMagnet();
    void ConstructGeometry();
    ClassDef(AtMagnet,1)

};

#endif //MAGNET_H

