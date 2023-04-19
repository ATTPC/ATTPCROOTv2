#include "AtMCFitter.h"

#include "AtClusterize.h" // for AtClusterize
#include "AtDigiPar.h"    // for AtDigiPar
#include "AtEvent.h"      // for AtEvent
#include "AtMCResult.h"
#include "AtPSA.h" // for AtPSA
#include "AtParameterDistribution.h"
#include "AtPatternEvent.h"     // for AtPatternEvent
#include "AtPulse.h"            // for AtPulse
#include "AtRawEvent.h"         // for AtRawEvent
#include "AtSimpleSimulation.h" // for AtSimpleSimulation
#include "AtSimulatedPoint.h"   // IWYU pragma: keep
#include "AtSpaceChargeModel.h"

#include <FairLogger.h>    // for LOG, Logger
#include <FairParSet.h>    // for FairParSet
#include <FairRunAna.h>    // for FairRunAna
#include <FairRuntimeDb.h> // for FairRuntimeDb

#include <TObject.h> // for TObject

using std::move;
namespace MCFitter {

AtMCFitter::AtMCFitter(SimPtr sim, ClusterPtr cluster, PulsePtr pulse)
   : fMap(pulse->GetMap()), fSim(move(sim)), fClusterize(move(cluster)), fPulse(move(pulse)),
     fResults([](const AtMCResult &a, const AtMCResult &b) { return a.fObjective < b.fObjective; }),
     fRawEventArray("AtRawEvent"), fEventArray("AtEvent")
{
}

AtMCFitter::ParamPtr AtMCFitter::GetParameter(const std::string &name) const
{
   if (fParameters.find(name) != fParameters.end()) {
      return fParameters.at(name);
   }
   return nullptr;
}

void AtMCFitter::Init()
{
   CreateParamDistros();

   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   auto fPar = dynamic_cast<AtDigiPar *>(rtdb->getContainer("AtDigiPar"));

   fPulse->SetParameters(fPar);
   fClusterize->GetParameters(fPar);
   if (fPSA)
      fPSA->Init();
   if (fSim->GetSpaceChargeModel())
      fSim->GetSpaceChargeModel()->LoadParameters(fPar);
}

void AtMCFitter::Exec(const AtPatternEvent &event)
{
   fRawEventArray.Clear();
   fEventArray.Clear();
   fResults.clear();
   SetParamDistributions(event);

   for (int i = 0; i < fNumIter; ++i) {

      auto result = DefineEvent();
      auto mcPoints = SimulateEvent(result);

      int idx = DigitizeEvent(mcPoints);
      double obj = ObjectiveFunction(event, idx);

      result.fIterNum = idx;
      result.fObjective = obj;
      result.Print();
      fResults.insert(result);
   }
}

int AtMCFitter::DigitizeEvent(const TClonesArray &points)
{
   // Event has been simulated and is sitting in the fSim
   auto vec = fClusterize->ProcessEvent(points);
   int eventIndex = fRawEventArray.GetEntries();
   auto *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray.ConstructedAt(eventIndex));
   auto *event = dynamic_cast<AtEvent *>(fEventArray.ConstructedAt(eventIndex));

   *rawEvent = fPulse->GenerateEvent(vec);
   if (fPSA)
      *event = fPSA->Analyze(*rawEvent);

   return eventIndex;
}

void AtMCFitter::FillResultArray(TClonesArray &resultArray) const
{
   resultArray.Clear();
   LOG(debug) << "Filling TCLonesArray with " << fResults.size() << " things";
   for (auto &res : fResults) {
      int idx = resultArray.GetEntries();
      auto toFill = dynamic_cast<AtMCResult *>(resultArray.ConstructedAt(idx));
      if (toFill == nullptr) {
         LOG(fatal) << "Failed to get the AtMCResult to fill in output TClonesArray!";
         return;
      }
      LOG(debug2) << "Filling TClonesArray at " << idx;
      *toFill = res;
   }
}

AtMCResult AtMCFitter::DefineEvent()
{
   AtMCResult result;
   for (auto &[name, distro] : fParameters)
      result.fParameters[name] = distro->Sample();
   return result;
}

} // namespace MCFitter
