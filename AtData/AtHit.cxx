#include "AtHit.h"

#include <Rtypes.h>

#include <H5Cpp.h>

#include <cmath>
#include <utility>
ClassImp(AtHit);

AtHit::AtHit(Int_t hitID) : AtHit(hitID, -1, XYZPoint(0, 0, -1000), -1) {}

AtHit::AtHit(Int_t hitID, Int_t PadNum, XYZPoint loc, Double_t charge)
   : fPadNum(PadNum), fHitID(hitID), fPosition(std::move(loc)), fCharge(charge)
{
}

AtHit::AtHit(Int_t padNum, XYZPoint loc, Double_t charge) : AtHit(-1, padNum, std::move(loc), charge) {}

std::unique_ptr<AtHit> AtHit::Clone()
{
   return std::make_unique<AtHit>(*this);
}

AtHit::XYZVector AtHit::GetPositionSigma() const
{
   return {std::sqrt(fPositionVariance.X()), std::sqrt(fPositionVariance.Y()), std::sqrt(fPositionVariance.Z())};
}

H5::CompType AtHit::GetHDF5Type()
{
   H5::CompType type(sizeof(AtHit_t));

   type.insertMember("x", HOFFSET(AtHit_t, x), H5::PredType::NATIVE_DOUBLE);               // NOLINT
   type.insertMember("y", HOFFSET(AtHit_t, y), H5::PredType::NATIVE_DOUBLE);               // NOLINT
   type.insertMember("z", HOFFSET(AtHit_t, z), H5::PredType::NATIVE_DOUBLE);               // NOLINT
   type.insertMember("t", HOFFSET(AtHit_t, t), H5::PredType::NATIVE_INT);                  // NOLINT
   type.insertMember("A", HOFFSET(AtHit_t, A), H5::PredType::NATIVE_DOUBLE);               // NOLINT
   type.insertMember("trackID", HOFFSET(AtHit_t, trackID), H5::PredType::NATIVE_INT);      // NOLINT
   type.insertMember("pointIDMC", HOFFSET(AtHit_t, pointIDMC), H5::PredType::NATIVE_INT);  // NOLINT
   type.insertMember("energyMC", HOFFSET(AtHit_t, energyMC), H5::PredType::NATIVE_DOUBLE); // NOLINT
   type.insertMember("elossMC", HOFFSET(AtHit_t, elossMC), H5::PredType::NATIVE_DOUBLE);   // NOLINT
   type.insertMember("angleMC", HOFFSET(AtHit_t, angleMC), H5::PredType::NATIVE_DOUBLE);   // NOLINT
   type.insertMember("AMC", HOFFSET(AtHit_t, AMC), H5::PredType::NATIVE_INT);              // NOLINT
   type.insertMember("ZMC", HOFFSET(AtHit_t, ZMC), H5::PredType::NATIVE_INT);              // NOLINT

   return type;
}
