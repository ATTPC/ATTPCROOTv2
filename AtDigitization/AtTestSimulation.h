#ifndef AtTestSimulation_h
#define AtTestSimulation_h

#include "AtSimpleSimulationGeneratorTask.h"

#include <Rtypes.h>

class TBuffer;
class TClass;
class TMemberInspector;

class AtTestSimulation : public AtSimpleSimulationGeneratorTask {
public:
   using AtSimpleSimulationGeneratorTask::AtSimpleSimulationGeneratorTask;
   ~AtTestSimulation() override = default;

   ClassDefOverride(AtTestSimulation, 3);
};

#endif /* AtTestSimulation_h */
