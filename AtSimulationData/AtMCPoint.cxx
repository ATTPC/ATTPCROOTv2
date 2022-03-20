/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtMCPoint.h"

#include <iostream>
using std::cout;
using std::endl;

// -----   Default constructor   -------------------------------------------
AtMCPoint::AtMCPoint() : FairMCPoint() {}
// -------------------------------------------------------------------------

// -----   Standard constructor   ------------------------------------------
AtMCPoint::AtMCPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t tof, Double_t length,
                     Double_t eLoss)
   : FairMCPoint(trackID, detID, pos, mom, tof, length, eLoss)
{
}

// -----   Standard constructor   ------------------------------------------
AtMCPoint::AtMCPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t tof, Double_t length,
                     Double_t eLoss, TString VolName, Int_t detCopyID, Double_t EIni, Double_t AIni, Int_t A, Int_t Z)
   : FairMCPoint(trackID, detID, pos, mom, tof, length, eLoss)
{
   fDetCopyID = detCopyID;
   fVolName = VolName;
   fEnergyIni = EIni;
   fAngleIni = AIni;
   fAiso = A;
   fZiso = Z;
}

// -------------------------------------------------------------------------

// -----   Destructor   ----------------------------------------------------
AtMCPoint::~AtMCPoint() {}
// -------------------------------------------------------------------------

// -----   Public method Print   -------------------------------------------
void AtMCPoint::Print(const Option_t *opt) const
{
   cout << "-I- AtMCPoint: AtMC point for track " << fTrackID << " in detector " << fDetectorID << endl;
   cout << "    Position (" << fX << ", " << fY << ", " << fZ << ") cm" << endl;
   cout << "    Momentum (" << fPx << ", " << fPy << ", " << fPz << ") GeV" << endl;
   cout << "    Time " << fTime << " ns,  Length " << fLength << " cm,  Energy loss " << fELoss * 1.0e06 << " keV"
        << endl;
}

ClassImp(AtMCPoint)
