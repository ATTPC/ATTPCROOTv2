#include "AtPulseTask.h"

#include "AtDigiPar.h"
#include "AtElectronicResponse.h"
#include "AtMCPoint.h"
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h"
#include "AtRawEvent.h"
#include "AtSimulatedPoint.h"

#include <FairLogger.h>
#include <FairParSet.h> // for FairParSet
#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <Math/Point2D.h>
#include <Math/Vector3D.h>
#include <TAxis.h>
#include <TClonesArray.h>
#include <TF1.h>
#include <TH1.h>
#include <TH2Poly.h>
#include <TMath.h>
#include <TObject.h>
#include <TRandom.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <utility>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

using namespace ElectronicResponse;
using XYPoint = ROOT::Math::XYPoint;

AtPulseTask::AtPulseTask(std::shared_ptr<AtPulse> pulse)
   : FairTask("AtPulseTask"), fPulse(pulse), fRawEventArray(TClonesArray("AtRawEvent", 1))
{
}

void AtPulseTask::SetParContainers()
{
   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = dynamic_cast<AtDigiPar *>(rtdb->getContainer("AtDigiPar"));
   if (fPar == nullptr)
      LOG(fatal) << "Could not get the parameter container "
                 << "AtDigiPar";
}

InitStatus AtPulseTask::Init()
{
   LOG(INFO) << "Initilization of AtPulseTask";
   FairRootManager *ioman = FairRootManager::Instance();

   fSimulatedPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtSimulatedPoint"));
   if (fSimulatedPointArray == nullptr) {
      LOG(fatal) << "ERROR: Cannot find fSimulatedPointArray array!";
      return kFATAL;
   }

   ioman->Register("AtRawEvent", "cbmsim", &fRawEventArray, fIsPersistent);

   fPulse->SetParameters(fPar);

   // Retrieve kinematics for each simulated point
   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtTpcPoint"));
   if (fMCPointArray == nullptr && fSaveMCInfo) {
      LOG(fatal) << "Cannot find fMCPointArray array!";
      return kFATAL;
   }
   if (fMCPointArray != nullptr)
      ioman->Register("AtTpcPoint", "cbmsim", fMCPointArray, fIsPersistentAtTpcPoint);

   LOG(info) << " AtPulseTask : Initialization of parameters complete!";
   return kSUCCESS;
}

void AtPulseTask::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtPulseTask";
   reset();

   Int_t nMCPoints = fSimulatedPointArray->GetEntries();
   LOG(info) << "AtPulseTask: Number of Points " << nMCPoints;

   // Distributing electron pulses among the pads
   // Create a vector of simPoints to pass
   std::vector<AtSimulatedPoint *> simPoints;
   for (Int_t i = 0; i < nMCPoints; i++) {
      simPoints.push_back(dynamic_cast<AtSimulatedPoint *>(fSimulatedPointArray->At(i)));
      if (fSaveMCInfo)
         FillPointsMap(simPoints.back());
   }
   auto rawEvent = fPulse->GenerateEvent(simPoints);

   LOG(info) << "...End of collection of electrons in this event." << std::endl;

   rawEvent.SetEventID(fEventID);
   if (fSaveMCInfo)
      rawEvent.SetSimMCPointMap(MCPointsMap);
   rawEvent.SetIsGood(true);

   new (fRawEventArray[0]) AtRawEvent(std::move(rawEvent));

   std::cout << "AtPulseTask Event ID : " << fEventID << "\n";
   ++fEventID;
}

void AtPulseTask::FillPointsMap(AtSimulatedPoint *point)
{
   auto pos = XYPoint(point->GetPosition().X(), point->GetPosition().Y());
   int padNum = fPulse->GetMap()->GetPadNum(pos);
   auto mcPoint = dynamic_cast<AtMCPoint *>(fMCPointArray->At(point->GetMCPointID()));
   saveMCInfo(point->GetMCPointID(), padNum, mcPoint->GetTrackID());
}

void AtPulseTask::saveMCInfo(int mcPointID, int padNumber, int trackID)
{
   // Count occurrences of simPoints coming from the same mcPoint
   int count = 0;

   // The same MC point ID is saved per pad only once, but duplicates are allowed in other pads
   for (auto it = MCPointsMap.lower_bound(padNumber); it != MCPointsMap.upper_bound(padNumber); ++it) {
      auto mcPointMap = dynamic_cast<AtMCPoint *>(fMCPointArray->At(mcPointID));
      auto trackIDMap = mcPointMap->GetTrackID();
      if (it->second == mcPointID || (trackID == trackIDMap)) {
         ++count;
         break;
      }
   }

   // insert if the mcPointID and trackID do not both match any existing point
   if (count == 0)
      MCPointsMap.insert(std::make_pair(padNumber, mcPointID));
}

void AtPulseTask::reset()
{
   MCPointsMap.clear();
   fRawEventArray.Delete();
}

ClassImp(AtPulseTask);
