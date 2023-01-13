#ifndef ATTABINFOEVENT_H
#define ATTABINFOEVENT_H

#include "AtTabInfoBase.h"

#include <TString.h>

class AtEvent;
class TClonesArray;
class AtEventManagerNew;

class AtTabInfoEvent : public AtTabInfoBase {
protected:
   AtEventManagerNew *fEventManager;

   TString fBranchName;

   TClonesArray *fEventArray{};

   AtEvent *fEvent;

public:
   AtTabInfoEvent();
   void Init() override;
   void Update() override;

   void SetBranch(TString branchName) { fBranchName = branchName; }

   AtEvent *GetEvent() { return fEvent; }

   ClassDefOverride(AtTabInfoEvent, 1);
};

#endif
