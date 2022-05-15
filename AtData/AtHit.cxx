#include "AtHit.h"

#include <Rtypes.h>

#include <utility>

ClassImp(AtHit);

AtHit::AtHit(Int_t hitID) : AtHit(hitID, -1, XYZPoint(0, 0, -1000), -1) {}

AtHit::AtHit(Int_t hitID, Int_t PadNum, XYZPoint loc, Double_t charge)
   : fPadNum(PadNum), fHitID(hitID), fPosition(std::move(loc)), fCharge(charge)
{
}

AtHit::AtHit(Int_t padNum, XYZPoint loc, Double_t charge) : AtHit(-1, padNum, std::move(loc), charge) {}

XYZVector AtHit::GetPositionSigma() const
{
   return {std::sqrt(fPositionVariance.X()), std::sqrt(fPositionVariance.Y()), std::sqrt(fPositionVariance.Z())};
}
