#ifndef ATTABINFOHEISTMUSIC_H
#define ATTABINFOHEISTMUSIC_H

#include "AtTabInfoBase.h"
#include "AtTabInfoFairRoot.h"

#include <TString.h>

#include "HTMusicIC.h"
#include "TTreeReaderValue.h"

class TTreeReader;
class AtEventManagerNew;
class AtRawEvent;

class AtTabInfoHEISTmusic : public AtTabInfoBase {
protected:
   AtEventManagerNew *fEventManager;

   TTreeReader *fReader;
   TTreeReaderValue<HTMusicIC> *fMusicReader;

   TString fTree;

   HTMusicIC *fMusicIC;

   std::unique_ptr<AtTabInfoFairRoot<AtRawEvent>> fInfoRawEvent;

public:
   AtTabInfoHEISTmusic();
   void Init() override;
   void Update() override;

   HTMusicIC *GetMusicIC() { return fMusicIC; }

   void SetTree(TString name) { fTree = name; }

   void SetRawEventBranch(TString name) { fInfoRawEvent->SetBranch(name); }

   ClassDefOverride(AtTabInfoHEISTmusic, 1);
};

#endif
