#ifndef AtTestSimulation_h
#define AtTestSimulation_h

#include "AtSimParticleCollector.h"
#include "AtSimpleSimulation.h" // for AtSimpleSimulation

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOver...

#include "FairTask.h"

#include <memory>  // for unique_ptr
#include <utility> // for move
#include <FairMCEventHeader.h>

class FairPrimaryGenerator;
class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief FairTask wrapper for AtSimpleSimulation.
 *
 * When a FairPrimaryGenerator is provided via SetPrimaryGenerator(), it is called each event to
 * generate particles. The generated particles are collected by AtSimParticleCollector (bypassing
 * the Geant4/VMC stack entirely) and forwarded to AtSimpleSimulation::SimulateParticle().
 *
 * When no generator is set, the task does nothing (no hardcoded particles).
 */
class AtTestSimulation : public FairTask {
protected:
   std::unique_ptr<AtSimpleSimulation> fSimulation{nullptr}; //!
   FairPrimaryGenerator *fPrimGen{nullptr};                  //!
   AtSimParticleCollector fCollector;                        //!

   // Owned MCEventHeader required by FairPrimaryGenerator::GenerateEvent()
   std::unique_ptr<FairMCEventHeader> fMCHeader; //!

public:
   AtTestSimulation(std::unique_ptr<AtSimpleSimulation> sim) : fSimulation(std::move(sim)) {}
   virtual ~AtTestSimulation() = default;

   /**
    * @brief Set the primary generator used to populate particles each event.
    *
    * The generator is NOT owned by this task — the caller retains ownership.
    * It must remain valid for the lifetime of the task.
    */
   void SetPrimaryGenerator(FairPrimaryGenerator *primGen) { fPrimGen = primGen; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *option) override;
   virtual void Finish() override {}
   AtSimpleSimulation *GetSimulation() { return fSimulation.get(); }

   ClassDefOverride(AtTestSimulation, 1);
};

#endif /* AtTestSimulation_h */
