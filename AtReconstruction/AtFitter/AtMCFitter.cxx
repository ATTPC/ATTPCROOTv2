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

#include <chrono>

using std::move;
namespace MCFitter {

AtMCFitter::AtMCFitter(SimPtr sim, ClusterPtr cluster, PulsePtr pulse)
   : fMap(pulse->GetMap()), fSim(move(sim)), fClusterize(move(cluster)), fPulse(move(pulse)),
     fResults([](const AtMCResult &a, const AtMCResult &b) { return a.fObjective < b.fObjective; })
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
   fRawEventArray.clear();
   fEventArray.clear();
   fResults.clear();

   SetParamDistributions(event);

   auto start = std::chrono::high_resolution_clock::now();
   for (int i = 0; i < fNumIter; ++i) {

      auto result = DefineEvent();
      auto mcPoints = SimulateEvent(result);

      int idx = DigitizeEvent(mcPoints);
      double obj = ObjectiveFunction(event, idx, result);

      result.fIterNum = idx;
      result.fObjective = obj;
      // result.Print();
      fResults.insert(result);
   }
   auto stop = std::chrono::high_resolution_clock::now();
   if (fTimeEvent)
      LOG(info) << "Simulation of " << fNumIter << " events took "
                << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms.";
}

int AtMCFitter::DigitizeEvent(const TClonesArray &points)
{
   // Event has been simulated and is sitting in the fSim
   auto vec = fClusterize->ProcessEvent(points);
   int eventIndex = fRawEventArray.size();
   LOG(info) << "Digitizing event at " << eventIndex;

   fRawEventArray.push_back(std::move(fPulse->GenerateEvent(vec)));
   if (fPSA)
      fEventArray.push_back(std::move(fPSA->Analyze(fRawEventArray.at(eventIndex))));

   return eventIndex;
}

/**
 * Fill the TClonesArray in order of smallest to largest chi2.
 */
void AtMCFitter::FillResultArrays(TClonesArray &resultArray, TClonesArray &simEvent, TClonesArray &simRawEvent)
{
   resultArray.Clear();
   simEvent.Clear();
   simRawEvent.Clear();

   for (auto &res : fResults) {

      int clonesIdx = resultArray.GetEntries();
      int eventIdx = res.fIterNum;
      LOG(info) << "Filling iteration " << eventIdx << " at index " << resultArray.GetEntries();

      auto result = dynamic_cast<AtMCResult *>(resultArray.ConstructedAt(clonesIdx));
      auto event = dynamic_cast<AtEvent *>(simEvent.ConstructedAt(clonesIdx));
      auto rawEvent = dynamic_cast<AtRawEvent *>(simRawEvent.ConstructedAt(clonesIdx));

      if (result == nullptr) {
         LOG(fatal) << "Failed to get the AtMCResult to fill in output TClonesArray!";
         return;
      }
      if (event == nullptr) {
         LOG(fatal) << "Failed to get the AtEvent to fill in output TClonesArray!";
         return;
      }
      if (rawEvent == nullptr) {
         LOG(fatal) << "Failed to get the AtRawEvent to fill in output TClonesArray!";
         return;
      }

      *result = res;
      *event = std::move(fEventArray[eventIdx]);
      *rawEvent = std::move(fRawEventArray[eventIdx]);
   }

   fEventArray.clear();
   fRawEventArray.clear();
}

AtMCResult AtMCFitter::DefineEvent()
{
   AtMCResult result;
   for (auto &[name, distro] : fParameters)
      result.fParameters[name] = distro->Sample();
   return result;
}

} // namespace MCFitter
