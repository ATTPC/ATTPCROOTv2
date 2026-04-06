#include "AtSimpleSimulationGeneratorTask.h"

#include "AtVertexPropagator.h"

#include <FairPrimaryGenerator.h>
#include <FairTask.h>

AtSimpleSimulationGeneratorTask::AtSimpleSimulationGeneratorTask(std::unique_ptr<AtSimpleSimulation> sim)
   : AtSimpleSimulationTask(std::move(sim))
{
}

InitStatus AtSimpleSimulationGeneratorTask::InitEventSource()
{
   if (fPrimGen == nullptr)
      return kSUCCESS;

   fMCHeader = std::make_unique<FairMCEventHeader>();
   fPrimGen->SetEvent(fMCHeader.get());
   fPrimGen->Init();
   return kSUCCESS;
}

AtSimpleSimulationTask::EventState AtSimpleSimulationGeneratorTask::LoadEvent()
{
   fCollector.Clear();
   if (fPrimGen == nullptr)
      return {};

   fPrimGen->GenerateEvent(&fCollector);
   EventState state;
   state.hasEvent = true;
   state.beamEvent = AtVertexPropagator::Instance()->IsBeamEvent();
   state.transportPrimaries = true;
   return state;
}

ClassImp(AtSimpleSimulationGeneratorTask);
