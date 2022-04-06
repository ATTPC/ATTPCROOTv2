#include "AtHit.h"

#include "Rtypes.h"

ClassImp(AtHit);

AtHit::AtHit(Int_t hitID) : AtHit(hitID, XYZPoint(0, 0, -1000), -1) {}

// Sets default padNum to -1
AtHit::AtHit(Int_t hitID, const XYZPoint &loc, Double_t charge) : AtHit(hitID, -1, loc, charge) {}
// Sets default padNum to -1
AtHit::AtHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge) : AtHit(hitID, XYZPoint(x, y, z), charge)
{
}

AtHit::AtHit(Int_t hitID, Int_t PadNum, const XYZPoint &loc, Double_t charge)
   : fPadNum(PadNum), fHitID(hitID), fPosition(loc), fCharge(charge)
{
}

AtHit::AtHit(Int_t hitID, Int_t padNum, Double_t x, Double_t y, Double_t z, Double_t charge)
   : AtHit(hitID, padNum, XYZPoint(x, y, z), charge)
{
}

/*void AtHit::SetClusterID(Int_t clusterID)
{
   fClusterID = clusterID;
   fIsClustered = kTRUE;
}
*/
