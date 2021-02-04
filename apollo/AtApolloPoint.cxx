/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtApolloPoint.h"

#include <iostream>
using std::cout;
using std::endl;

// -----   Default constructor   -------------------------------------------
AtApolloPoint::AtApolloPoint()
  : FairMCPoint()
{
}
// -------------------------------------------------------------------------

// -----   Standard constructor   ------------------------------------------
AtApolloPoint::AtApolloPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom,
                                   Int_t crystalID,
                                   Double_t tof, Double_t length,
                                   Double_t eLoss)
  : FairMCPoint(trackID, detID,  pos, mom, tof, length, eLoss)
{
}

// -------------------------------------------------------------------------

// -----   Destructor   ----------------------------------------------------
AtApolloPoint::~AtApolloPoint() { }
// -------------------------------------------------------------------------

// -----   Public method Print   -------------------------------------------
void AtApolloPoint::Print(const Option_t* opt) const
{
  cout << "-I- AtApolloPoint: AtApollo point for track " << fTrackID
       << " in detector " << fDetectorID  << " and crystal "<< fCrystalID << endl;
  cout << "    Time " << fTime << " ns,  Length " << fLength
       << " cm,  Energy loss " << fELoss*1.0e06 << " keV" << endl;
}
// -------------------------------------------------------------------------

ClassImp(AtApolloPoint)
