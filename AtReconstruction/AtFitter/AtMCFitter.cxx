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
#include <mutex>
#include <thread>
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
   fPar = dynamic_cast<AtDigiPar *>(rtdb->getContainer("AtDigiPar"));

   fPulse->SetParameters(fPar);
   fClusterize->GetParameters(fPar);
   if (fPSA)
      fPSA->Init();
   if (fSim->GetSpaceChargeModel())
      fSim->GetSpaceChargeModel()->LoadParameters(fPar);

   fThPulse.resize(fNumThreads);
   for (int i = 0; i < fNumThreads; ++i)
      fThPulse[i] = fPulse->Clone();
   fThPSA.clear();
   for (int i = 0; i < fNumThreads; ++i)
      fThPSA.emplace_back(std::move(fPSA->Clone()));
}

void AtMCFitter::RunIterRange(int startIter, int numIter, AtPulse *pulse, AtPSA *psa)
{
   // Here we should copy each thread their own version of the clusterize, pulse, and simulation
   // objects (only if the number of threads is greater than 1). Needs to be deep copies

   for (int i = 0; i < numIter; ++i) {

      int idx = startIter + i;
      auto result = DefineEvent();
      auto mcPoints = SimulateEvent(result);

      DigitizeEvent(mcPoints, idx, pulse, psa);
      double obj = ObjectiveFunction(*fCurrentEvent, idx, result);

      result.fIterNum = idx;
      result.fObjective = obj;
      // result.Print();
      {
         std::lock_guard<std::mutex> lk(fResultMutex);
         fResults.insert(result);
      }
   }
   LOG(debug) << "Done with run iter range";
}

void AtMCFitter::Exec(const AtPatternEvent &event)
{
   fRawEventArray.clear();
   fEventArray.clear();
   fResults.clear();

   SetParamDistributions(event);

   auto start = std::chrono::high_resolution_clock::now();

   // Make sure the event arrays are large enough so no resizing will happen
   fRawEventArray.resize(fNumIter);
   fEventArray.resize(fNumIter);

   // Get what iterations to do on what thread.
   std::vector<std::pair<int, int>> threadParam;
   int iterPerTh = fNumIter / fNumThreads;
   for (int i = 0; i < fNumThreads; ++i)
      threadParam.emplace_back(0, iterPerTh);
   for (int i = 0; i < fNumIter % fNumThreads; ++i)
      threadParam[i].second++;
   for (int i = 1; i < fNumThreads; ++i)
      threadParam[i].first = threadParam[i - 1].first + threadParam[i - 1].second;

   for (int i = 0; i < threadParam.size(); ++i) {
      LOG(info) << i << ": " << threadParam[i].first << " " << threadParam[i].second;
   }

   // Set the conditions for simulating the event
   fCurrentEvent = &event;

   std::vector<std::thread> threads;
   for (int i = 0; i < fNumThreads; ++i) {
      LOG(debug) << "Creating thread " << i << " with " << threadParam[i].first << " " << threadParam[i].second
                 << " and " << fPulse.get();
      threads.emplace_back([this](std::pair<int, int> param, AtPulse *pulse,
                                  AtPSA *psa) { this->RunIterRange(param.first, param.second, pulse, psa); },
                           threadParam[i], fThPulse[i].get(), fPSA.get());
   }

   for (auto &th : threads)
      th.join();

   auto stop = std::chrono::high_resolution_clock::now();

   if (fTimeEvent)
      LOG(info) << "Simulation of " << fNumIter << " events took "
                << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms.";
}

int AtMCFitter::DigitizeEvent(const TClonesArray &points, int idx, AtPulse *pulse, AtPSA *psa)
{
   // Event has been simulated and is sitting in the fSim
   auto vec = fClusterize->ProcessEvent(points);
   LOG(debug) << "Digitizing event at " << idx;

   fRawEventArray[idx] = pulse->GenerateEvent(vec);

   if (psa) {
      LOG(debug) << "Running PSA at " << idx << " with " << psa;
      fEventArray[idx] = psa->Analyze(fRawEventArray[idx]);
   }
   LOG(debug) << "Done digitizing event at " << idx;
   return idx;
}

/**
 * Fill the TClonesArray in order of smallest to largest chi2.
 */
void AtMCFitter::FillResultArrays(TClonesArray &resultArray, TClonesArray &simEvent, TClonesArray &simRawEvent)
{
   resultArray.Delete();
   simEvent.Delete();
   simRawEvent.Delete();

   for (auto &res : fResults) {

      int clonesIdx = resultArray.GetEntries();
      int eventIdx = res.fIterNum;
      LOG(debug) << "Filling iteration " << eventIdx << " at index " << resultArray.GetEntries();

      new (resultArray[clonesIdx]) AtMCResult(std::move(res));
      if (clonesIdx < fNumEventsToSave) {
         new (simEvent[clonesIdx]) AtEvent(std::move(fEventArray[eventIdx]));
         new (simRawEvent[clonesIdx]) AtRawEvent(std::move(fRawEventArray[eventIdx]));
      }
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
