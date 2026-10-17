#ifndef AtSimTransportReplayTask_h
#define AtSimTransportReplayTask_h

#include "AtSimTransportTask.h"

#include <Rtypes.h>

#include <string>

class TBuffer;
class TClass;
class TFile;
class TMemberInspector;
class TTree;

class AtSimTransportReplayTask : public AtSimTransportTask {
public:
   explicit AtSimTransportReplayTask(std::unique_ptr<AtSimTransport> sim);
   ~AtSimTransportReplayTask() override = default;

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

   ClassDefOverride(AtSimTransportReplayTask, 1);
};

#endif
