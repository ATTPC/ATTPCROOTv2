#ifndef AtSimTransportTask_h
#define AtSimTransportTask_h

#include "AtSimParticleCollector.h"
#include "AtSimTransport.h"

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
class TParticle;

class AtSimTransportTask : public FairTask {
public:
   struct EventState {
      bool hasEvent{false};
      bool beamEvent{false};
      bool transportPrimaries{true};
   };

   explicit AtSimTransportTask(std::unique_ptr<AtSimTransport> sim);
   ~AtSimTransportTask() override = default;

   void SetSensitiveDetector(AtTpc *detector) { fDetector = detector; }
   void SetDetector(AtTpc *detector) { SetSensitiveDetector(detector); }

   InitStatus Init() override;
   void Exec(Option_t *option) override;
   void Finish() override;

   /// Enable/disable automatic field extraction from FairRun. On by default for drop-in behavior.
   void SetAutoConfigureField(bool enable) { fAutoConfigureField = enable; }

   AtSimTransport *GetSimulation() { return fSimulation.get(); }

protected:
   std::unique_ptr<AtSimTransport> fSimulation{nullptr}; //!
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
   void TransportParticle(const TParticle &particle, int trackID, bool beamEvent);
   /// Build an AtTpc::StepState from a TransportStep and submit it to the detector.
   /// Returns true if transport should continue (false if detector requested stop).
   /// When exiting, position/momentum reference the pre-step state; otherwise post-step.
   bool SubmitDetectorStep(const AtSimTransport::TransportStep &step, int trackID, bool beamTrack, bool entering,
                           bool exiting);
   ROOT::Math::XYZPoint FindSensitiveEntry(const ROOT::Math::XYZPoint &pos, const ROOT::Math::PxPyPzEVector &mom) const;
   bool IsSensitiveVolume(const std::string &volumeName) const;

   ClassDefOverride(AtSimTransportTask, 1);
};

#endif
