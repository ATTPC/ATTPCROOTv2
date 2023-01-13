#ifndef ATTABINFO_H
#define ATTABINFO_H

#include "AtTabInfoBase.h"

#include <TString.h>

#include <string>
#include <map>

class AtRawEvent;
class AtEvent;
class TClonesArray;
class AtEventManagerNew;

class AtTabInfo : public AtTabInfoBase {
protected:
   std::map<std::string, std::unique_ptr<AtTabInfoBase>> fInfoAugments;

public:
   AtTabInfo();
   void Init() override;
   void Update() override;

   AtTabInfoBase *AddAugment(std::string name, std::unique_ptr<AtTabInfoBase> augment);
   AtTabInfoBase *ReplaceAugment(std::string name, std::unique_ptr<AtTabInfoBase> augment);
   AtTabInfoBase *GetAugment(std::string name);

   ClassDefOverride(AtTabInfo, 1);
};

#endif
