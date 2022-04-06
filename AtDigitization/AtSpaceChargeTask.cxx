#include "AtSpaceChargeTask.h"

#include "AtMCPoint.h"
#include "AtDigiPar.h"

#include <FairLogger.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

#include "TClonesArray.h"

AtSpaceChargeTask::AtSpaceChargeTask() : FairTask("AtSpaceChargeTask") {}

AtSpaceChargeTask::AtSpaceChargeTask(const char *name) : FairTask(name) {}

AtSpaceChargeTask::~AtSpaceChargeTask() {}

InitStatus AtSpaceChargeTask::Init()
{
   FairRootManager *ioman = FairRootManager::Instance();

   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject(fInputBranchName));
   if (fMCPointArray == nullptr) {
      LOG(fatal) << "Cannot find fMCPointArray array!";
      return kERROR;
   }

   fSpaceChargePointArray = new TClonesArray(fOutputBranchName);
   ioman->Register("AtSimulatedPoint", "cbmsim", fSpaceChargePointArray, fIsPersistent);

   getParameters();

   return kSUCCESS;
}

void AtSpaceChargeTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtClusterizeTask";

   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = dynamic_cast<AtDigiPar *>(rtdb->getContainer("AtDigiPar"));
   if (fPar == nullptr)
      LOG(fatal) << "Could not get the parameter container "
                 << "AtDigiPar";
}

void AtSpaceChargeTask::getParameters()
{
   fVelDrift = fPar->GetDriftVelocity(); // [cm/us]
   fDetPadPlane = fPar->GetZPadPlane();  //[mm]

   std::cout << "  Drift velocity: " << fVelDrift << std::endl;
   std::cout << "  Position of the pad plane (Z): " << fDetPadPlane << std::endl;
}

void AtSpaceChargeTask::Exec(Option_t *opt)
{
   fSpaceChargePointArray->Clear();

   for (int i = 0; i < fMCPointArray->GetEntries(); ++i) {
      auto mcPoint = dynamic_cast<AtMCPoint *>(fMCPointArray->At(i));
      if (mcPoint->GetVolName() == "drift_volume")
         processPoint(mcPoint);
      else
         LOG(info) << "Skipping point " << i << ". Not in drift volume.";
   }
}

void AtSpaceChargeTask::processPoint(AtMCPoint *point)
{
   // Create a copy of the point
}
