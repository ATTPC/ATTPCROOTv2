#ifndef AtSimpleSimulationReplayTask_h
#define AtSimpleSimulationReplayTask_h

#include "AtSimpleSimulationTask.h"

#include <Rtypes.h>

#include <string>

class TBuffer;
class TClass;
class TFile;
class TMemberInspector;
class TTree;

class AtSimpleSimulationReplayTask : public AtSimpleSimulationTask {
public:
   explicit AtSimpleSimulationReplayTask(std::unique_ptr<AtSimpleSimulation> sim);
   ~AtSimpleSimulationReplayTask() override = default;

   void SetPrimaryTrackSource(const std::string &fileName) { fPrimaryTrackSourceFile = fileName; }

protected:
   std::string fPrimaryTrackSourceFile;      //!
   TFile *fPrimaryTrackFile{nullptr};        //!
   TTree *fPrimaryTrackTree{nullptr};        //!
   TClonesArray *fPrimaryTrackInput{nullptr};//!
   Long64_t fSourceEventIndex{0};            //!

   InitStatus InitEventSource() override;
   EventState LoadEvent() override;
   void FinishEventSource() override;

private:
   bool LoadPrimaryTracksFromSource();

   ClassDefOverride(AtSimpleSimulationReplayTask, 1);
};

#endif
