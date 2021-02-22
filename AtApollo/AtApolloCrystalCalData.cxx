/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

#include "AtApolloCrystalCalData.h"

#include <iostream>

using std::cout;
using std::endl;
using std::flush;

AtApolloCrystalCalData::AtApolloCrystalCalData()
    : FairMultiLinkedData()
    , fEnergy(-1.)
    , fTime(0)
    , fCrystalId(-1)
{
}

AtApolloCrystalCalData::AtApolloCrystalCalData(Int_t ident,
                                                 Double_t energy,
                                                 ULong64_t time)
    : FairMultiLinkedData()
    , fEnergy(energy)
    , fTime(time)
    , fCrystalId(ident)
{
}

AtApolloCrystalCalData::AtApolloCrystalCalData(const AtApolloCrystalCalData& right)
    : FairMultiLinkedData(right)
    , fEnergy(right.fEnergy)
    , fTime(right.fTime)
    , fCrystalId(right.fCrystalId)
{
}

void AtApolloCrystalCalData::Print(const Option_t* opt) const
{
    cout << "-I- AtApolloCrystalCalData: a crystalCalData level hit in crystal identifier " << fCrystalId << endl;
    cout << "    Energy = " << fEnergy << " (GeV in sim)" << endl;
    cout << "    Time =" << fTime << " ns  " << endl;
}
