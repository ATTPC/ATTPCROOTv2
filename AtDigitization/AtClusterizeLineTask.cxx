#include "AtClusterizeLineTask.h"

#include "FairLogger.h"
#include "FairRootManager.h"

#include "TClonesArray.h"
#include "Math/Vector3D.h"

#include "AtDigiPar.h"
#include "AtTpcPoint.h"
#include "AtSimulatedLine.h"

AtClusterizeLineTask::AtClusterizeLineTask() : AtClusterizeTask("AtClusterizeLineTask")
{}

AtClusterizeLineTask::~AtClusterizeLineTask() {}

InitStatus AtClusterizeLineTask::Init() 
{
   FairRootManager *ioman = FairRootManager::Instance();

   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtTpcPoint"));
   if (fMCPointArray == nullptr)
      LOG(fatal) << "Cannot find AtTpcPoint array!";

   fSimulatedPointArray = new TClonesArray("AtSimulatedLine");
   ioman->Register("AtSimulatedPoint", "cbmsim", fSimulatedPointArray, fIsPersistent);

   getParameters();

   return kSUCCESS;
}

void AtClusterizeLineTask::getParameters()
{
   AtClusterizeTask::getParameters();
   fTBTime = fPar->GetTBTime() / 1000.;    // in us
   std::cout << "  TB width: " << fTBTime << std::endl;
   
}


void AtClusterizeLineTask::processPoint(Int_t mcPointID)
{
   auto trackID = fMCPoint->GetTrackID();

   // If it is a new track entering the volume or no energy was deposited
   // record its location and the new track ID
   if (fMCPoint->GetEnergyLoss() == 0 || fCurrTrackID != trackID) {
      setNewTrack();
      std::cout << "Point is new track" << std::endl;
      return;
   }

   ROOT::Math::XYZVector currentPoint = getCurrentPointLocation();
   
   auto size = fSimulatedPointArray->GetEntriesFast();
   AtSimulatedLine *simLine = new ((*fSimulatedPointArray)[size]) AtSimulatedLine();

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
