#include "AtMCFitter.h"

#include "AtClusterize.h"
#include "AtDigiPar.h"
#include "AtEvent.h"
#include "AtPSA.h"
#include "AtPatternEvent.h"
#include "AtPulse.h"
#include "AtRawEvent.h"
#include "AtSimpleSimulation.h"
#include "AtSimulatedPoint.h"

#include <FairRunAna.h>
#include <FairRuntimeDb.h>
namespace MCFitter {

AtMCFitter::AtMCFitter(SimPtr sim, ClusterPtr cluster, PulsePtr pulse)
   : fMap(pulse->GetMap()), fSim(sim), fClusterize(cluster), fPulse(pulse),
     fObjectives([](const ObjPair &a, const ObjPair &b) { return a.second < b.second; }), fRawEventArray("AtRawEvent"),
     fEventArray("AtEvent")
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
}

void AtMCFitter::Exec(const AtPatternEvent &event)
{
   fRawEventArray.Clear();
   fEventArray.Clear();
   SetParamsFromEvent(event);

   for (int i = 0; i < fNumIter; ++i) {
      SimulateEvent();
      int idx = DigitizeEvent();
      double obj = ObjectiveFunction(event, idx);
      fObjectives.insert({idx, obj});
   }
}

int AtMCFitter::DigitizeEvent()
{
   // Event has been simulated and is sitting in the fSim
   auto vec = fClusterize->ProcessEvent(fSim->GetPointsArray());
   int eventIndex = fRawEventArray.GetEntries();
   AtRawEvent *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray.ConstructedAt(eventIndex));
   AtEvent *event = dynamic_cast<AtEvent *>(fEventArray.ConstructedAt(eventIndex));

   *rawEvent = fPulse->GenerateEvent(vec);
   if (fPSA)
      *event = fPSA->Analyze(*rawEvent);

   return eventIndex;
}

} // namespace MCFitter
