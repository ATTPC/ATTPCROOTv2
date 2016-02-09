/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

// -------------------------------------------------------------------------
// -----                    AtPipe file                                -----
// -----                Created by M. Al-Turany  June 2014             -----
// -------------------------------------------------------------------------

#ifndef PIPE_H
#define PIPE_H

#include "FairModule.h"

class AtPipe : public FairModule {
  public:
    AtPipe(const char * name, const char *Title="At Pipe");
    AtPipe();

    virtual ~AtPipe();
    virtual void ConstructGeometry();
   
  ClassDef(AtPipe,1) //AtPIPE

};

#endif //PIPE_H

