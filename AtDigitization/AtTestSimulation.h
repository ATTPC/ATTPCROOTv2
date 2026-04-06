#ifndef AtTestSimulation_h
#define AtTestSimulation_h

#include "AtSimParticleCollector.h"
#include "AtMCTrack.h"
#include "AtSimpleSimulation.h" // for AtSimpleSimulation

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOver...
#include <TClonesArray.h>

#include "FairTask.h"

#include <memory>  // for unique_ptr
#include <utility> // for move
#include <FairMCEventHeader.h>

class AtTpc;
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
   AtTpc *fDetector{nullptr};                                //!
   AtSimParticleCollector fCollector;                        //!
   TClonesArray *fMCTrackArray{nullptr};                     //!

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
   void SetDetector(AtTpc *detector) { fDetector = detector; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *option) override;
   virtual void Finish() override {}
   AtSimpleSimulation *GetSimulation() { return fSimulation.get(); }

private:
   void RegisterMCTrackBranch();
   void FillMCTracks();
   bool ProcessDetectorStep(const AtSimpleSimulation::TransportStep &step, int trackID, bool beamTrack, bool preSensitive,
                            bool postSensitive, bool entering, bool exiting);
   ROOT::Math::XYZPoint FindSensitiveEntry(const ROOT::Math::XYZPoint &pos, const ROOT::Math::PxPyPzEVector &mom) const;
   static bool IsSensitiveVolume(const std::string &volumeName);

   ClassDefOverride(AtTestSimulation, 2);
};

#endif /* AtTestSimulation_h */
