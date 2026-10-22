#include "AtFittedTrackOld.h"

#include <Rtypes.h>

#include <iterator>
#include <numeric>

ClassImp(AtFittedTrackOld);

using XYZVector = ROOT::Math::XYZVector;

const std::tuple<Float_t, Float_t, Float_t, Float_t, Float_t, Float_t, Float_t> AtFittedTrackOld::GetEnergyAngles()
{
   return std::forward_as_tuple(fEnergy, fEnergyXtr, fTheta, fPhi, fEnergyPRA, fThetaPRA, fPhiPRA);
}

const std::tuple<XYZVector, XYZVector, XYZVector> AtFittedTrackOld::GetVertices()
{
   return std::forward_as_tuple(fInitialPos, fInitialPosPRA, fInitialPosXtr);
}

const std::tuple<Float_t, Float_t, Float_t, Float_t, Float_t, Bool_t> AtFittedTrackOld::GetStats()
{
   return std::forward_as_tuple(fPValue, fChi2, fBChi2, fNdf, fBNdf, fFitConverged);
}

const std::tuple<Int_t, Float_t, Float_t, Float_t, std::string, Int_t> AtFittedTrackOld::GetTrackProperties()
{
   return std::forward_as_tuple(fCharge, fBrho, fELossADC, fDEdxADC, fPDG, fTrackPoints);
}

const std::tuple<Float_t, Float_t> AtFittedTrackOld::GetIonChamber()
{
   return std::forward_as_tuple(fIonChamberEnergy, fIonChamberTime);
}

const std::tuple<Float_t, Float_t> AtFittedTrackOld::GetExcitationEnergy()
{
   return std::forward_as_tuple(fExcitationEnergy, fExcitationEnergyXtr);
}

const std::tuple<Float_t, Float_t, Float_t> AtFittedTrackOld::GetDistances()
{
   return std::forward_as_tuple(fDistanceXtr, fTrackLength, fPOCAXtr);
}
