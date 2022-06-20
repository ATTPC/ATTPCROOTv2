#include "AtSiPoint.h"

#include <TVector3.h>

#include <iostream>

using std::cout;
using std::endl;

AtSiPoint::AtSiPoint() : AtMCPoint() {}

AtSiPoint::AtSiPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t tof, Double_t length,
                     Double_t eLoss)
   : AtMCPoint(trackID, detID, pos, mom, tof, length, eLoss)
{
}

AtSiPoint::AtSiPoint(Int_t trackID, Int_t detID, TVector3 posIn, TVector3 posOut, TVector3 momIn, TVector3 momOut,
                     Double_t tof, Double_t length, Double_t eLoss, TString VolName, Int_t detCopyID, Double_t EIni,
                     Double_t AIni, Int_t A, Int_t Z)
   : AtMCPoint(trackID, detID, posIn, posOut, tof, length, eLoss, VolName, detCopyID, EIni, AIni, A, Z)
{
   SetPositionOut(posOut);
   SetMomentumOut(momOut);
}

void AtSiPoint::Print(const Option_t *opt) const
{
   cout << "-I- AtSiPoint: AtSiArray point for track " << fTrackID << " in detector " << fDetectorID << endl;
   cout << "    Position (" << fX << ", " << fY << ", " << fZ << ") cm" << endl;
   cout << "    Momentum (" << fPx << ", " << fPy << ", " << fPz << ") GeV" << endl;
   cout << "    Time " << fTime << " ns,  Length " << fLength << " cm,  Energy loss " << fELoss * 1.0e06 << " keV"
        << endl;
}
// -------------------------------------------------------------------------

// -----   Point x coordinate from linear extrapolation   ------------------
Double_t AtSiPoint::GetX(Double_t z) const
{
   //  cout << fZ << " " << z << " " << fZ_out << endl;
   if ((fZ_out - z) * (fZ - z) >= 0.)
      return (fX_out + fX) / 2.;
   Double_t dz = fZ_out - fZ;
   return (fX + (z - fZ) / dz * (fX_out - fX));
}
// -------------------------------------------------------------------------

// -----   Point y coordinate from linear extrapolation   ------------------
Double_t AtSiPoint::GetY(Double_t z) const
{
   if ((fZ_out - z) * (fZ - z) >= 0.)
      return (fY_out + fY) / 2.;
   Double_t dz = fZ_out - fZ;
   //  if ( TMath::Abs(dz) < 1.e-3 ) return (fY_out+fY)/2.;
   return (fY + (z - fZ) / dz * (fY_out - fY));
}

ClassImp(AtSiPoint)
