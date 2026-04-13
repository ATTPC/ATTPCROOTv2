#include "AtSimTransportGeneratorTask.h"

#include "AtVertexPropagator.h"

#include <FairPrimaryGenerator.h>
#include <FairTask.h>

AtSimTransportGeneratorTask::AtSimTransportGeneratorTask(std::unique_ptr<AtSimTransport> sim)
   : AtSimTransportTask(std::move(sim))
{
}

InitStatus AtSimTransportGeneratorTask::InitEventSource()
{
   if (fPrimGen == nullptr)
      return kSUCCESS;

   fMCHeader = std::make_unique<FairMCEventHeader>();
   fPrimGen->SetEvent(fMCHeader.get());
   fPrimGen->Init();
   return kSUCCESS;
}

AtSimTransportTask::EventState AtSimTransportGeneratorTask::LoadEvent()
{
   fCollector.Clear();
   if (fPrimGen == nullptr)
      return {};

   // Capture the beam flag before GenerateEvent(), because AtReactionGenerator::ReadEvent()
   // calls EndEvent() which toggles the flag before returning.
   bool wasBeamEvent = AtVertexPropagator::Instance()->IsBeamEvent();
   fPrimGen->GenerateEvent(&fCollector);
   EventState state;
   state.hasEvent = true;
   state.beamEvent = wasBeamEvent;
   state.transportPrimaries = true;
   return state;
}

ClassImp(AtSimTransportGeneratorTask);
