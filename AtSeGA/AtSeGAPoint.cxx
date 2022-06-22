/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtSeGAPoint.h"

#include <iostream>
using std::cout;
using std::endl;

// -----   Default constructor   -------------------------------------------
AtSeGAPoint::AtSeGAPoint() : FairMCPoint() {}
// -------------------------------------------------------------------------

// -----   Standard constructor   ------------------------------------------
AtSeGAPoint::AtSeGAPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Int_t crystalID, Double_t tof,
                             Double_t length, Double_t eLoss)
   : FairMCPoint(trackID, detID, pos, mom, tof, length, eLoss)
{
   fCrystalID = crystalID;

}

// -------------------------------------------------------------------------

// -----   Destructor   ----------------------------------------------------
AtSeGAPoint::~AtSeGAPoint() {}
// -------------------------------------------------------------------------

// -----   Public method Print   -------------------------------------------
void AtSeGAPoint::Print(const Option_t *opt) const
{
   cout << "-I- AtSeGAPoint: AtSeGA point for track " << fTrackID << " in detector " << fDetectorID
        << " and crystal " << fCrystalID << endl;
   cout << "    Time " << fTime << " ns,  Length " << fLength << " cm,  Energy loss " << fELoss * 1.0e06 << " keV"
        << endl;
}
// -------------------------------------------------------------------------

ClassImp(AtSeGAPoint)
