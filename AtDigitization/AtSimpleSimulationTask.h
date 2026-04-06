#ifndef AtSimpleSimulationTask_h
#define AtSimpleSimulationTask_h

#include "AtSimParticleCollector.h"
#include "AtSimpleSimulation.h"

#include <FairMCEventHeader.h>
#include <FairTask.h>
#include <Rtypes.h>

#include <memory>
#include <string>

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

   InitStatus Init() override;
   void Exec(Option_t *option) override;
   void Finish() override;

   AtSimpleSimulation *GetSimulation() { return fSimulation.get(); }

protected:
   std::unique_ptr<AtSimpleSimulation> fSimulation{nullptr}; //!
   AtTpc *fDetector{nullptr};                                //!
   AtSimParticleCollector fCollector;                        //!
   TClonesArray *fMCTrackArray{nullptr};                     //!

   virtual InitStatus InitEventSource();
   virtual EventState LoadEvent() = 0;
   virtual void FinishEventSource();

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
