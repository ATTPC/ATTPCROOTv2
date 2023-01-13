#ifndef ATTABINFORAWEVENT_H
#define ATTABINFORAWEVENT_H

#include "AtTabInfoBase.h"

#include <TString.h>

class AtRawEvent;
class TClonesArray;
class AtEventManagerNew;

class AtTabInfoRawEvent : public AtTabInfoBase {
protected:
   AtEventManagerNew *fEventManager;

   TString fBranchName;

   TClonesArray *fRawEventArray{};

   AtRawEvent *fRawEvent;

public:
   AtTabInfoRawEvent();
   void Init() override;
   void Update() override;

   void SetBranch(TString branchName) { fBranchName = branchName; }

   AtRawEvent *GetRawEvent() { return fRawEvent; }

   ClassDefOverride(AtTabInfoRawEvent, 1);
};

#endif
