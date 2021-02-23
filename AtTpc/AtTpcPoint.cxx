/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtTpcPoint.h"

#include <iostream>
using std::cout;
using std::endl;


// -----   Default constructor   -------------------------------------------
AtTpcPoint::AtTpcPoint()
  : FairMCPoint()
{
}
// -------------------------------------------------------------------------

// -----   Standard constructor   ------------------------------------------
AtTpcPoint::AtTpcPoint(Int_t trackID, Int_t detID,
                                   TVector3 pos, TVector3 mom,
                                   Double_t tof, Double_t length,
                                   Double_t eLoss)
  : FairMCPoint(trackID, detID, pos, mom, tof, length, eLoss)
{
}

// -----   Standard constructor   ------------------------------------------
AtTpcPoint::AtTpcPoint(Int_t trackID, Int_t detID, TString VolName, Int_t detCopyID,
			 TVector3 posIn,
			 TVector3 posOut, TVector3 momIn, TVector3 momOut,
			 Double_t tof, Double_t length, Double_t eLoss, Double_t EIni, Double_t AIni,Int_t A, Int_t Z)
  : FairMCPoint(trackID, detID, posIn, momIn, tof, length, eLoss) {
  fDetCopyID = detCopyID;
  fX_out     = posOut.X();
  fY_out     = posOut.Y();
  fZ_out     = posOut.Z();
  fPx_out    = momOut.Px();
  fPy_out    = momOut.Py();
  fPz_out    = momOut.Pz();
  fVolName   = VolName;
  fEnergyIni = EIni;
  fAngleIni  = AIni;
  fAiso      = A;
  fZiso      = Z;
}



// -------------------------------------------------------------------------

// -----   Destructor   ----------------------------------------------------
AtTpcPoint::~AtTpcPoint() { }
// -------------------------------------------------------------------------

// -----   Public method Print   -------------------------------------------
void AtTpcPoint::Print(const Option_t* opt) const
{
  cout << "-I- AtTpcPoint: AtTpc point for track " << fTrackID
       << " in detector " << fDetectorID << endl;
  cout << "    Position (" << fX << ", " << fY << ", " << fZ
       << ") cm" << endl;
  cout << "    Momentum (" << fPx << ", " << fPy << ", " << fPz
       << ") GeV" << endl;
  cout << "    Time " << fTime << " ns,  Length " << fLength
       << " cm,  Energy loss " << fELoss*1.0e06 << " keV" << endl;
}
// -------------------------------------------------------------------------

// -----   Point x coordinate from linear extrapolation   ------------------
Double_t AtTpcPoint::GetX(Double_t z) const {
  //  cout << fZ << " " << z << " " << fZ_out << endl;
  if ( (fZ_out-z)*(fZ-z) >= 0. ) return (fX_out+fX)/2.;
  Double_t dz = fZ_out - fZ;
  return ( fX + (z-fZ) / dz * (fX_out-fX) );
}
// -------------------------------------------------------------------------



// -----   Point y coordinate from linear extrapolation   ------------------
Double_t AtTpcPoint::GetY(Double_t z) const {
  if ( (fZ_out-z)*(fZ-z) >= 0. ) return (fY_out+fY)/2.;
  Double_t dz = fZ_out - fZ;
  //  if ( TMath::Abs(dz) < 1.e-3 ) return (fY_out+fY)/2.;
  return ( fY + (z-fZ) / dz * (fY_out-fY) );
}
// -------------------------------------------------------------------------

ClassImp(AtTpcPoint)
