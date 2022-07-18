#include "AtClusterizeLineTask.h"

#include "AtDigiPar.h"
#include "AtMCPoint.h"
#include "AtSimulatedLine.h"

#include <FairLogger.h>
#include <FairRootManager.h>

#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <TClonesArray.h>
#include <TObject.h>

#include <iostream>
#include <memory>

AtClusterizeLineTask::AtClusterizeLineTask() : AtClusterizeTask("AtClusterizeLineTask") {}

AtClusterizeLineTask::~AtClusterizeLineTask() = default;

InitStatus AtClusterizeLineTask::Init()
{
   FairRootManager *ioman = FairRootManager::Instance();

   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtTpcPoint"));
   if (fMCPointArray == nullptr)
      LOG(fatal) << "Cannot find AtTpcPoint array!";

   fSimulatedPointArray = std::make_unique<TClonesArray>("AtSimulatedLine");
   ioman->Register("AtSimulatedPoint", "cbmsim", fSimulatedPointArray.get(), fIsPersistent);

   getParameters();

   return kSUCCESS;
}

void AtClusterizeLineTask::getParameters()
{
   AtClusterizeTask::getParameters();
   fTBTime = fPar->GetTBTime() / 1000.; // in us
   std::cout << "  TB width: " << fTBTime << std::endl;
}

void AtClusterizeLineTask::processPoint(Int_t mcPointID)
{
   auto trackID = fMCPoint->GetTrackID();

   // If it is a new track entering the volume or no energy was deposited
   // record its location and the new track ID
   if (fMCPoint->GetEnergyLoss() == 0 || fCurrTrackID != trackID) {
      setNewTrack();
      return;
   }

   ROOT::Math::XYZVector currentPoint = getCurrentPointLocation();

   auto size = fSimulatedPointArray->GetEntriesFast();
   auto *simLine = new ((*fSimulatedPointArray)[size]) AtSimulatedLine(); // NOLINT

   simLine->SetMCPointID(mcPointID);
   simLine->SetMCEventID(fMCPoint->GetEventID());
   simLine->SetClusterID(0);

   simLine->SetInitialPosition(fPrevPoint.x(), fPrevPoint.y(), fPrevPoint.z());
   simLine->SetFinalPosition(currentPoint.x(), currentPoint.y(), currentPoint.z());

   simLine->SetCharge(getNumberOfElectronsGenerated());

   simLine->SetTransverseDiffusion(getTransverseDiffusion(simLine->GetPosition().z()));
   simLine->SetLongitudinalDiffusion(getLongitudinalDiffusion(simLine->GetPosition().z()));

   fPrevPoint = currentPoint;
}

ClassImp(AtClusterizeLineTask);
