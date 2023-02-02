#include "AtDataManip.h"

#include "AtDigiPar.h"

#include <FairLogger.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <TF1.h>

#include <AtHit.h>

/**
 * Assumes that window is at z = 0, and the electrons are drifting towards the pad plane at
 * the z locations specified in the parameter file.
 */
double AtTools::GetTB(double z)
{
   auto fPar = dynamic_cast<AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return -1;
   }

   auto driftDistance = fPar->GetZPadPlane() - z;
   auto driftTB = fPar->GetTBEntrance() - GetDriftTB(driftDistance);
   LOG(debug) << driftDistance << " " << driftTB;

   return driftTB;
}

double AtTools::GetDriftTB(double d)
{
   auto fPar = dynamic_cast<AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return -1;
   }
   auto driftTime = d / (fPar->GetDriftVelocity() * 10.);
   return driftTime / (fPar->GetTBTime() / 1000.);
}
std::unique_ptr<TF1> AtTools::GetHitFunction(const AtHit &hit)
{
   if (hit.GetPositionSigma().Z() == 0) {
      LOG(error) << "Hits that are points (sig_z = 0) are not supported yet!";
      return nullptr;
   }

   // position in mm to TB.
   auto fPar = dynamic_cast<AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));

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

std::unique_ptr<TF1> AtTools::GetHitFunctionTB(const AtHit &hit)
{
   if (hit.GetPositionSigma().Z() == 0) {
      LOG(error) << "Hits that are points (sig_z = 0) are not supported yet!";
      return nullptr;
   }

   // position in mm to TB.
   auto fPar = dynamic_cast<AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));

   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return nullptr;
   }

   // Create the function we are going to set and make sure it isn't added to the global list
   auto func = std::make_unique<TF1>("hitFuncTB", "gaus", 0, 512, TF1::EAddToList::kNo);

   auto ampl = hit.GetCharge() / (GetDriftTB(hit.GetPositionSigma().Z()) * std::sqrt(2 * TMath::Pi()));
   func->SetParameter(0, ampl);
   func->SetParameter(1, GetTB(hit.GetPosition().Z()));
   func->SetParameter(2, GetDriftTB(hit.GetPositionSigma().Z()));

   return func;
}
