#include "AtDataManip.h"

#include "AtDigiPar.h"

#include <FairLogger.h>
#include <FairParSet.h> // for FairParSet
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <TF1.h>
#include <TMath.h> // for Pi

#include <AtHit.h>

#include <cmath> // for sqrt

/**
 * Assumes that window is at z = 0, and the electrons are drifting towards the pad plane at
 * the z locations specified in the parameter file.
 */
double AtTools::GetTB(double z, const AtDigiPar *fPar)
{
   if (fPar == nullptr)
      fPar = dynamic_cast<const AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return -1;
   }

   auto driftDistance = fPar->GetZPadPlane() - z;
   auto driftTB = fPar->GetTBEntrance() - GetDriftTB(driftDistance, fPar);
   LOG(debug) << driftDistance << " " << driftTB;

   return driftTB;
}

double AtTools::GetDriftTB(double d, const AtDigiPar *fPar)
{
   if (fPar == nullptr)
      fPar = dynamic_cast<const AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return -1;
   }
   auto driftTime = d / (fPar->GetDriftVelocity() * 10.);
   return driftTime / (fPar->GetTBTime() / 1000.);
}
std::unique_ptr<TF1> AtTools::GetHitFunction(const AtHit &hit, const AtDigiPar *fPar)
{
   if (hit.GetPositionSigma().Z() == 0) {
      LOG(error) << "Hits that are points (sig_z = 0) are not supported yet!";
      return nullptr;
   }

   // position in mm to TB.
   if (fPar == nullptr)
      fPar = dynamic_cast<const AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));

   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return nullptr;
   }

   // Create the function we are going to set and make sure it isn't added to the global list
   auto func = std::make_unique<TF1>("hitFunc", "gaus", 0, fPar->GetZPadPlane(), TF1::EAddToList::kNo);
   auto ampl = hit.GetCharge() / (hit.GetPositionSigma().Z() * std::sqrt(2 * TMath::Pi()));
   func->SetParameter(0, ampl);
   func->SetParameter(1, hit.GetPosition().Z());
   func->SetParameter(2, hit.GetPositionSigma().Z());

   return func;
}

std::unique_ptr<TF1> AtTools::GetHitFunctionTB(const AtHit &hit, const AtDigiPar *fPar)
{
   if (hit.GetPositionSigma().Z() == 0) {
      LOG(error) << "Hits that are points (sig_z = 0) are not supported yet!";
      return nullptr;
   }
   if (fPar == nullptr)
      fPar = dynamic_cast<const AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));

   // Create the function we are going to set and make sure it isn't added to the global list
   auto func = std::make_unique<TF1>("hitFuncTB", "gaus", 0, 512, TF1::EAddToList::kNo);

   auto ampl = hit.GetCharge() / (GetDriftTB(hit.GetPositionSigma().Z(), fPar) * std::sqrt(2 * TMath::Pi()));
   func->SetParameter(0, ampl);
   func->SetParameter(1, GetTB(hit.GetPosition().Z(), fPar));
   func->SetParameter(2, GetDriftTB(hit.GetPositionSigma().Z(), fPar));

   return func;
}
