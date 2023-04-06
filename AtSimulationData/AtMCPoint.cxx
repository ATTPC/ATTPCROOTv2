/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtMCPoint.h"

#include <FairMCPoint.h>

#include <TVector3.h>

#include <iostream>
#include <utility>

using std::cout;
using std::endl;

AtMCPoint::AtMCPoint() : FairMCPoint() {}

AtMCPoint::AtMCPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t tof, Double_t length,
                     Double_t eLoss)
   : FairMCPoint(trackID, detID, pos, mom, tof, length, eLoss)
{
}

AtMCPoint::AtMCPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t tof, Double_t length,
                     Double_t eLoss, TString VolName, Int_t detCopyID, Double_t EIni, Double_t AIni, Int_t A, Int_t Z)
   : FairMCPoint(trackID, detID, pos, mom, tof, length, eLoss), fDetCopyID(detCopyID), fVolName(std::move(VolName)),
     fEnergyIni(EIni), fAngleIni(AIni), fAiso(A), fZiso(Z)
{
}

AtMCPoint::AtMCPoint(Int_t trackID, Int_t detID, XYZPoint pos, XYZVector mom, Double_t tof, Double_t length,
                     Double_t eLoss)
   : AtMCPoint(trackID, detID, TVector3(pos.X(), pos.Y(), pos.Z()), TVector3(mom.X(), mom.Y(), mom.Z()), 0, length,
               eLoss)
{
}

void AtMCPoint::Print(const Option_t *opt) const
{
   cout << "-I- AtMCPoint: AtMC point for track " << fTrackID << " in detector " << fDetectorID << endl;
   cout << "    Position (" << fX << ", " << fY << ", " << fZ << ") cm" << endl;
   cout << "    Momentum (" << fPx << ", " << fPy << ", " << fPz << ") GeV" << endl;
   cout << "    Time " << fTime << " ns,  Length " << fLength << " cm,  Energy loss " << fELoss * 1.0e06 << " keV"
        << endl;
}

void AtMCPoint::Clear(Option_t *)
{
   // Clear FairMCPoint
   fTrackID = 0;
   fEventId = 0;
   fPx = 0;
   fPy = 0;
   fPz = 0;
   fTime = 0;
   fLength = 0;
   fELoss = 0;
   fDetectorID = 0;
   fX = 0;
   fY = 0;
   fZ = 0;

   // Clear AtMCPoint
   fDetCopyID = 0;
   fVolName = "";
   fEnergyIni = 0;
   fAngleIni = 0;
   fAiso = 0;
   fZiso = 0;
}

ClassImp(AtMCPoint)
