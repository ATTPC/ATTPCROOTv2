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

/*void AtHit::SetClusterID(Int_t clusterID)
{
   fClusterID = clusterID;
   fIsClustered = kTRUE;
}
*/
