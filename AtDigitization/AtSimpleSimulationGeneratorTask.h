#ifndef AtSimpleSimulationGeneratorTask_h
#define AtSimpleSimulationGeneratorTask_h

#include "AtSimpleSimulationTask.h"

#include <FairMCEventHeader.h>
#include <Rtypes.h>

#include <memory>

class FairPrimaryGenerator;
class TBuffer;
class TClass;
class TMemberInspector;

class AtSimpleSimulationGeneratorTask : public AtSimpleSimulationTask {
public:
   explicit AtSimpleSimulationGeneratorTask(std::unique_ptr<AtSimpleSimulation> sim);
   ~AtSimpleSimulationGeneratorTask() override = default;

   void SetPrimaryGenerator(FairPrimaryGenerator *primGen) { fPrimGen = primGen; }
   void SetEventGenerator(FairPrimaryGenerator *primGen) { SetPrimaryGenerator(primGen); }

protected:
   FairPrimaryGenerator *fPrimGen{nullptr};                  //!
   std::unique_ptr<FairMCEventHeader> fMCHeader{nullptr};    //!

   InitStatus InitEventSource() override;
   EventState LoadEvent() override;

   ClassDefOverride(AtSimpleSimulationGeneratorTask, 1);
};

#endif
