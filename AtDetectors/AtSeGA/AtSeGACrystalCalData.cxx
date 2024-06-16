/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

#include "AtSeGACrystalCalData.h"

#include <FairMultiLinkedData.h> // for FairMultiLinkedData

#include <Rtypes.h> // for TGenericClassInfo

#include <iostream>

using std::cout;
using std::endl;
using std::flush;
ClassImp(AtSeGACrystalCalData);

AtSeGACrystalCalData::AtSeGACrystalCalData() : FairMultiLinkedData(), fEnergy(-1.), fTime(0), fDetCopyID(-1) {}

AtSeGACrystalCalData::AtSeGACrystalCalData(Int_t ident, Double_t energy, ULong64_t time)
   : FairMultiLinkedData(), fEnergy(energy), fTime(time), fDetCopyID(ident)
{
}

void AtSeGACrystalCalData::Print(const Option_t *opt) const
{
   cout << "-I- AtSeGACrystalCalData: a crystalCalData level hit in crystal identifier " << fDetCopyID << endl;
   cout << "    Energy = " << fEnergy << " (GeV in sim)" << endl;
   cout << "    Time =" << fTime << " ns  " << endl;
}
