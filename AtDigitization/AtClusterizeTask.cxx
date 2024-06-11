#include "AtClusterizeTask.h"

#include "AtClusterize.h" // for AtClusterize
#include "AtDigiPar.h"
#include "AtSimulatedPoint.h" // IWYU pragma: keep

#include <FairLogger.h>
#include <FairParSet.h>
#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <Math/Point3D.h>     // for PositionVector3D
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacementVector3D
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <TClonesArray.h>
#include <TObject.h>

#include <memory>
#include <string> // for string
#include <utility>

using XYZVector = ROOT::Math::XYZVector;
using XYZPoint = ROOT::Math::XYZPoint;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

AtClusterizeTask::AtClusterizeTask(std::shared_ptr<AtClusterize> clusterize, const char *name)
   : FairTask(name), fClusterize(std::move(clusterize))
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
   LOG(info) << "Initilization of AtClusterizeTask";

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
