/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

// -------------------------------------------------------------------------
// -----                    AtFieldCreator header file                  -----
// -----                Created 26/03/14  by M. Al-Turany              -----
// -------------------------------------------------------------------------


#ifndef AtFieldCreator_H
#define AtFieldCreator_H

#include "FairFieldFactory.h"

class AtFieldPar;

class FairField;

class AtFieldCreator : public FairFieldFactory 
{

 public:
  AtFieldCreator();
  virtual ~AtFieldCreator();
  virtual FairField* createFairField();
  virtual void SetParm();
  ClassDef(AtFieldCreator,1);
  
 protected:
  AtFieldPar* fFieldPar;
  
 private:
  AtFieldCreator(const AtFieldCreator&);
  AtFieldCreator& operator=(const AtFieldCreator&);

};
#endif //AtFieldCreator_H
