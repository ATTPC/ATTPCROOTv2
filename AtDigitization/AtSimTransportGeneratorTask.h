#ifndef AtSimTransportGeneratorTask_h
#define AtSimTransportGeneratorTask_h

#include "AtSimTransportTask.h"

#include <FairMCEventHeader.h>
#include <Rtypes.h>

#include <memory>

class FairPrimaryGenerator;
class TBuffer;
class TClass;
class TMemberInspector;

class AtSimTransportGeneratorTask : public AtSimTransportTask {
public:
   explicit AtSimTransportGeneratorTask(std::unique_ptr<AtSimTransport> sim);
   ~AtSimTransportGeneratorTask() override = default;

   void SetPrimaryGenerator(FairPrimaryGenerator *primGen) { fPrimGen = primGen; }
   void SetEventGenerator(FairPrimaryGenerator *primGen) { SetPrimaryGenerator(primGen); }

protected:
   FairPrimaryGenerator *fPrimGen{nullptr};                  //!
   std::unique_ptr<FairMCEventHeader> fMCHeader{nullptr};    //!

   InitStatus InitEventSource() override;
   EventState LoadEvent() override;

   ClassDefOverride(AtSimTransportGeneratorTask, 1);
};

#endif
