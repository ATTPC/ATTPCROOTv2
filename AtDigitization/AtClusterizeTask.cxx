#include "AtClusterizeTask.h"

#include <FairLogger.h>
#include <FairParSet.h>
#include <FairTask.h>

#include <TMathBase.h>
#include <TObject.h>
#include <TRandom.h>
#include <TString.h>

#include <iostream>
#include <memory>

// Fair class header
#include "AtDigiPar.h"
#include "AtMCPoint.h"
#include "AtSimulatedPoint.h"

#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

#include <TClonesArray.h>
#include <TMath.h>

using XYZVector = ROOT::Math::XYZVector;
using XYZPoint = ROOT::Math::XYZPoint;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

AtClusterizeTask::AtClusterizeTask(std::shared_ptr<AtClusterize> clusterize, const char *name)
   : FairTask(name), fClusterize(clusterize)
{
}

AtClusterizeTask::~AtClusterizeTask()
{
   LOG(debug) << "Destructor of AtClusterizeTask";
}

void AtClusterizeTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtClusterizeTask";

   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = dynamic_cast<AtDigiPar *>(rtdb->getContainer("AtDigiPar"));
   if (fPar == nullptr)
      LOG(fatal) << "Could not get the parameter container "
                 << "AtDigiPar";
}
InitStatus AtClusterizeTask::Init()
{
   LOG(INFO) << "Initilization of AtClusterizeTask";

   FairRootManager *ioman = FairRootManager::Instance();

   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtTpcPoint"));
   if (fMCPointArray == nullptr) {
      LOG(error) << "Cannot find fMCPointArray array!";
      return kERROR;
   }

   fSimulatedPointArray = std::make_unique<TClonesArray>(fClusterize->GetSavedClassName().c_str());
   ioman->Register("AtSimulatedPoint", "cbmsim", fSimulatedPointArray.get(), fIsPersistent);

   fClusterize->GetParameters(fPar);
   return kSUCCESS;
}

void AtClusterizeTask::Exec(Option_t *option)
{
   fSimulatedPointArray->Delete();

   auto points = fClusterize->ProcessEvent(*fMCPointArray);
   fClusterize->FillTClonesArray(*fSimulatedPointArray, points);
}

ClassImp(AtClusterizeTask);
