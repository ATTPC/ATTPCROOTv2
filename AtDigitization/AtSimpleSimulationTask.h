#ifndef AtSimpleSimulationTask_h
#define AtSimpleSimulationTask_h

#include "AtSimParticleCollector.h"
#include "AtSimpleSimulation.h"

#include <FairMCEventHeader.h>
#include <FairTask.h>
#include <Rtypes.h>

#include <memory>
#include <string>

namespace AtTools {
class AtELossModelFactory;
} // namespace AtTools

class AtTpc;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

class AtSimpleSimulationTask : public FairTask {
public:
   struct EventState {
      bool hasEvent{false};
      bool beamEvent{false};
      bool transportPrimaries{true};
   };

   explicit AtSimpleSimulationTask(std::unique_ptr<AtSimpleSimulation> sim);
   ~AtSimpleSimulationTask() override = default;

   void SetSensitiveDetector(AtTpc *detector) { fDetector = detector; }
   void SetDetector(AtTpc *detector) { SetSensitiveDetector(detector); }

   /// Set a model factory for automatic energy loss model creation. See AtSimpleSimulation::SetModelFactory.
   void SetModelFactory(std::shared_ptr<AtTools::AtELossModelFactory> factory)
   {
      fSimulation->SetModelFactory(std::move(factory));
   }

   InitStatus Init() override;
   void Exec(Option_t *option) override;
   void Finish() override;

   /// Enable/disable automatic field extraction from FairRun. On by default for drop-in behavior.
   void SetAutoConfigureField(bool enable) { fAutoConfigureField = enable; }

   AtSimpleSimulation *GetSimulation() { return fSimulation.get(); }

protected:
   std::unique_ptr<AtSimpleSimulation> fSimulation{nullptr}; //!
   AtTpc *fDetector{nullptr};                                //!
   bool fAutoConfigureField{true};                           //!
   AtSimParticleCollector fCollector;                        //!
   TClonesArray *fMCTrackArray{nullptr};                     //!

   virtual InitStatus InitEventSource();
   virtual EventState LoadEvent() = 0;
   virtual void FinishEventSource();
   void ConfigureFieldFromFairRun();

   void RegisterMCTrackBranch();
   void FillMCTracks();
   void TransportCurrentEvent(bool beamEvent);
   void TransportParticle(const AtCollectedParticle &particle, bool beamEvent);
   bool SubmitInitialSensitivePoint(int trackID, int pdg, bool beamTrack, const ROOT::Math::XYZPoint &pos,
                                    const ROOT::Math::PxPyPzEVector &mom);
   bool ProcessDetectorStep(const AtSimpleSimulation::TransportStep &step, int trackID, bool beamTrack, bool preSensitive,
                            bool postSensitive, bool entering, bool exiting);
   ROOT::Math::XYZPoint FindSensitiveEntry(const ROOT::Math::XYZPoint &pos,
                                           const ROOT::Math::PxPyPzEVector &mom) const;
   static bool IsSensitiveVolume(const std::string &volumeName);

   ClassDefOverride(AtSimpleSimulationTask, 1);
};

#endif
