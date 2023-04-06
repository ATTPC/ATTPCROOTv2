#ifndef AtTestSimulation_h
#define AtTestSimulation_h

#include "AtSimpleSimulation.h" // for AtSimpleSimulation

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOver...

#include "FairTask.h"

#include <memory>  // for unique_ptr
#include <utility> // for move
class TBuffer;
class TClass;
class TMemberInspector;

class AtTestSimulation : public FairTask {
protected:
   std::unique_ptr<AtSimpleSimulation> fSimulation{nullptr}; //!

public:
   AtTestSimulation(std::unique_ptr<AtSimpleSimulation> sim) : fSimulation(std::move(sim)) {}
   virtual ~AtTestSimulation() = default;

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *option) override;
   virtual void Finish() override {}
   AtSimpleSimulation *GetSimulation() { return fSimulation.get(); }

   ClassDefOverride(AtTestSimulation, 1);
};

#endif /* AtTestSimulation_h */
